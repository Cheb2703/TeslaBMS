#include "Packhealth.h"
#include <Preferences.h>
#include <ArduinoJson.h>
#include <math.h>
#include "config.h"
#include "BMSModuleManager.h"
#include "Blackbox.h"
#include "Celldrift.h"

// See Packhealth.h for what this is and why.

namespace {

const int HIST = 10;   // charge sessions / estimates kept

struct Saved {
    uint8_t wireCount;
    uint8_t capCount;
    uint8_t warnWiring;
    uint8_t offsetKnown;
    float   offsetV;       // charger reading minus the cells added together, at 0 A
    float   wire[HIST];    // mOhm per charge session, oldest first
    float   cap[HIST];     // Ah per estimate, oldest first
};

// A finished (or power-cut) charge waiting for the next power-on's rested reading.
struct Pending {
    uint32_t session;      // power-on the charge happened in
    uint8_t  valid;
    uint8_t  startKnown;   // a rested reading was taken just before it
    float    startMeanV;
    float    ah;
};

Saved    s;
Pending  s_waiting;            // from an earlier power-on, loaded at boot
bool     s_waitingActive = false;
volatile bool s_resetPending = false;
bool     s_warnRestored = false;

// Rested reading tracker (charger off, pack not moving). s_restMs = 0: none.
float    s_restMeanV = 0.0f;
uint32_t s_restMs = 0;

// Zero-offset calibration, this power-on
float    s_offSum = 0.0f;
int      s_offN = 0;

// The charge session in progress
bool     s_chgActive = false;
bool     s_chgStartKnown = false;
float    s_chgStartMeanV = 0.0f;
float    s_chgAh = 0.0f;
double   s_wireSumDv = 0.0, s_wireSumI = 0.0;
int      s_wireN = 0;
uint32_t s_lastTickMs = 0;
uint32_t s_lastPendingSaveMs = 0;

// Charge level from rested voltage: the same NCA table the display uses
// (socFractionFromOcv() in Displaymanager.cpp). Approximate -- published curves
// differ by a few percent, which is why only big charges are used and the
// median of several estimates is shown.
float ocvSoc(float v) { return socFractionFromOcv(v); }

float median(const float* src, int n) {
    float v[HIST];
    for (int i = 0; i < n; i++) v[i] = src[i];
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && v[j] < v[j - 1]; j--) { float t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
    return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

void push(float* hist, uint8_t& count, float value) {
    if (count < HIST) { hist[count++] = value; return; }
    for (int i = 1; i < HIST; i++) hist[i - 1] = hist[i];
    hist[HIST - 1] = value;
}

void saveAll() {
    Preferences p;
    p.begin("packhealth", false);
    p.putBytes("saved", &s, sizeof(s));
    p.end();
}

void savePending(const Pending& pend) {
    Preferences p;
    p.begin("packhealth", false);
    p.putBytes("pending", &pend, sizeof(pend));
    p.end();
}

void clearPending() {
    Pending none;
    memset(&none, 0, sizeof(none));
    savePending(none);
}

bool meanCellV(const DisplayData& dd, float& out) {
    int cells = (dd.numModules < 2 ? dd.numModules : 2) * 6;
    if (cells == 0) return false;
    float sum = 0.0f;
    for (int i = 0; i < cells; i++) {
        float v = dd.cellVolt[i / 6][i % 6];
        if (!(v > 1.0f && v < 5.0f)) return false;
        sum += v;
    }
    out = sum / cells;
    return true;
}

void finishCapacity(float afterMeanV) {
    s_waitingActive = false;
    clearPending();
    if (!s_waiting.startKnown) {
        Blackbox::log("CAP", "No capacity estimate: there was no rested reading just before the charge");
        return;
    }
    float a = ocvSoc(s_waiting.startMeanV), b = ocvSoc(afterMeanV), step = b - a;
    if (step < CAPACITY_MIN_SOC_STEP) {
        Blackbox::log("CAP", "No capacity estimate: the charge was too small (%.0f%% -> %.0f%%, %.1f Ah in)",
                      a * 100.0f, b * 100.0f, s_waiting.ah);
        return;
    }
    float capAh = s_waiting.ah / step;
    push(s.cap, s.capCount, capAh);
    saveAll();
    Blackbox::log("CAP", "Capacity estimate %.0f Ah: %.1f Ah in, rested %.3f -> %.3f V per cell (%.0f%% -> %.0f%%)",
                  capAh, s_waiting.ah, s_waiting.startMeanV, afterMeanV, a * 100.0f, b * 100.0f);
}

void finishWiring(BMSModuleManager& bms) {
    if (!s.offsetKnown) {
        Blackbox::log("WIRE", "No wiring figure: the zero offset hasn't been measured yet (charger off, pack at rest)");
        return;
    }
    if (s_wireN < WIRING_MIN_SAMPLES) return;   // too short, or never steady -- nothing to say
    float r = (float)(s_wireSumDv / s_wireSumI) * 1000.0f;   // mOhm
    bool haveUsual = s.wireCount >= 3;
    float usual = haveUsual ? median(s.wire, s.wireCount) : NAN;
    bool warn = haveUsual && (r - usual >= WIRING_WARN_RISE_MOHM) && (r >= usual * (1.0f + WIRING_WARN_RISE_FRAC));
    push(s.wire, s.wireCount, r);
    if (haveUsual) {
        s.warnWiring = warn ? 1 : 0;
        if (warn) bms.reportFault("WIRING", "CHARGE WIRING RESISTANCE UP", false, true);
        else      bms.clearFaultById("WIRING");
    }
    saveAll();
    if (haveUsual) Blackbox::log("WIRE", "Charge wiring resistance %.1f mOhm (usual %.1f)", r, usual);
    else           Blackbox::log("WIRE", "Charge wiring resistance %.1f mOhm (building up a usual value)", r);
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

void Packhealth::begin() {
    memset(&s, 0, sizeof(s));
    memset(&s_waiting, 0, sizeof(s_waiting));
    Preferences p;
    p.begin("packhealth", true);
    if (p.getBytes("saved", &s, sizeof(s)) != sizeof(s)) memset(&s, 0, sizeof(s));
    if (p.getBytes("pending", &s_waiting, sizeof(s_waiting)) != sizeof(s_waiting)) memset(&s_waiting, 0, sizeof(s_waiting));
    p.end();
    if (s.wireCount > HIST || s.capCount > HIST) memset(&s, 0, sizeof(s));
    // A charge from an earlier power-on is waiting for this power-on's rested reading.
    s_waitingActive = s_waiting.valid && s_waiting.session != Blackbox::session();
}

void Packhealth::poll(const DisplayData& dd, int chargeEvent, bool chargerOnline, bool chargerOn,
                      float chargerVolts, float chargerAmps, int steadyState, BMSModuleManager& bms) {
    uint32_t now = millis();
    uint32_t dt = s_lastTickMs ? now - s_lastTickMs : 0;
    s_lastTickMs = now;

    if (s_resetPending) {
        s_resetPending = false;
        float off = s.offsetV;
        uint8_t offKnown = s.offsetKnown;
        memset(&s, 0, sizeof(s));
        s.offsetV = off;             // the zero offset is a property of the instruments, keep it
        s.offsetKnown = offKnown;
        saveAll();
        bms.clearFaultById("WIRING");
        Blackbox::log("NOTE", "Capacity and wiring history reset from the web UI");
    }

    if (!s_warnRestored) {
        s_warnRestored = true;
        if (s.warnWiring) bms.reportFault("WIRING", "CHARGE WIRING RESISTANCE UP", false, true);
    }

    float meanV;
    bool haveMean = meanCellV(dd, meanV);

    // Rested reading: charger off and the pack not moving. The first readings
    // after power-on count too (the pack sat idle while the board was off).
    // Only updated while the charger is off: switching it on makes the pack
    // "move", and that must not wipe the reading taken just before the charge.
    if (!chargerOn) {
        if (steadyState == 2) s_restMs = 0;                       // the lift moved it
        else if (haveMean) { s_restMeanV = meanV; s_restMs = now; }
    }

    // Zero offset: charger reading minus the cells, charger off, pack steady.
    // Measured once per power-on (20 s of readings) and saved.
    if (s_offN < 20 && chargerOnline && !chargerOn && steadyState == 1
        && chargerVolts > 1.0f && dd.packVoltage > 1.0f) {
        s_offSum += chargerVolts - dd.packVoltage;
        if (++s_offN == 20) {
            s.offsetV = s_offSum / 20.0f;
            s.offsetKnown = 1;
            saveAll();
        }
    }

    // A charge from an earlier power-on: finish it with this power-on's rested reading.
    if (s_waitingActive) {
        float after;
        if (chargerOn) {
            s_waitingActive = false;
            clearPending();
            Blackbox::log("CAP", "No capacity estimate: the charger was running at power-on, or started before a rested reading");
        } else if (Celldrift::restedMeanV(after)) {
            finishCapacity(after);
        } else if (now > DRIFT_SNAPSHOT_WINDOW_MS + 5000) {
            s_waitingActive = false;   // no rested reading this power-on (lift used straight away)
            clearPending();
        }
    }

    if (chargeEvent == 1) {
        s_chgActive = true;
        s_chgStartKnown = (s_restMs != 0 && now - s_restMs < 60000UL);   // rested within the minute before
        s_chgStartMeanV = s_restMeanV;
        s_chgAh = 0.0f;
        s_wireSumDv = s_wireSumI = 0.0;
        s_wireN = 0;
        s_lastPendingSaveMs = 0;
    }

    if (s_chgActive) {
        if (chargerOn && chargerAmps > 0.0f) s_chgAh += chargerAmps * (float)dt / 3600000.0f;
        if (chargerOn && chargerAmps >= WIRING_MIN_CURRENT_A && steadyState == 1 && s.offsetKnown
            && chargerVolts > 1.0f && dd.packVoltage > 1.0f) {
            s_wireSumDv += chargerVolts - dd.packVoltage - s.offsetV;
            s_wireSumI  += chargerAmps;
            s_wireN++;
        }
        // Keep the amp-hours saved while charging, so a charge cut off by the
        // board losing power still counts at the next power-on.
        if (chargeEvent == 1 || now - s_lastPendingSaveMs >= 120000UL || chargeEvent == 2) {
            s_lastPendingSaveMs = now;
            Pending pend;
            pend.session    = Blackbox::session();
            pend.valid      = 1;
            pend.startKnown = s_chgStartKnown ? 1 : 0;
            pend.startMeanV = s_chgStartMeanV;
            pend.ah         = s_chgAh;
            savePending(pend);
        }
    }

    if (chargeEvent == 2 && s_chgActive) {
        s_chgActive = false;
        finishWiring(bms);
    }
}

void Packhealth::requestReset() { s_resetPending = true; }

String Packhealth::json() {
    JsonDocument doc;
    doc["offsetKnown"] = s.offsetKnown != 0;
    doc["offsetMv"]    = serialized(String(s.offsetV * 1000.0f, 0));

    JsonObject w = doc["wiring"].to<JsonObject>();
    w["n"]    = s.wireCount;
    w["warn"] = s.warnWiring != 0;
    if (s.wireCount) {
        w["last"]  = serialized(String(s.wire[s.wireCount - 1], 1));
        w["usual"] = serialized(String(median(s.wire, s.wireCount), 1));
    }

    JsonObject c = doc["capacity"].to<JsonObject>();
    c["n"] = s.capCount;
    if (s.capCount) {
        c["last"]   = serialized(String(s.cap[s.capCount - 1], 0));
        c["median"] = serialized(String(median(s.cap, s.capCount), 0));
    }
    c["charging"]   = s_chgActive;
    c["sessionAh"]  = serialized(String(s_chgAh, 1));
    c["waiting"]    = s_waitingActive;   // an earlier charge still needs this power-on's rested reading

    String out;
    serializeJson(doc, out);
    return out;
}
