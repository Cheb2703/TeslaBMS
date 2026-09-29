#include "Celldrift.h"
#include <Preferences.h>
#include <ArduinoJson.h>
#include <math.h>
#include "config.h"
#include "BMSModuleManager.h"
#include "Blackbox.h"

// See Celldrift.h for what this is and why.

namespace {

const int MAX_CELLS = 12;   // DisplayData carries 2 modules x 6 cells

// One rested reading. Offsets are in 0.1 mV so they fit an int16.
struct Sample {
    uint16_t meanMv;     // average cell voltage, mV -- the charge level it was taken at
    uint16_t bledMask;   // cells balanced between the previous reading and this one
    int16_t  off[MAX_CELLS];
};

struct History {
    uint8_t  count;                 // readings stored (up to DRIFT_HISTORY)
    uint8_t  head;                  // slot the next reading goes into
    uint16_t warnMask;              // cells currently warned about
    int16_t  warnDrop[MAX_CELLS];   // how far behind, 0.1 mV, for re-raising the warning at boot
    Sample   s[DRIFT_HISTORY];
};

History  s_hist;
uint16_t s_bledMask = 0;   // cells balanced since the last reading; saved as it grows
bool     s_done = false;   // this power-on's reading is taken (or given up on)
bool     s_warningsRestored = false;
volatile bool s_resetPending = false;

// The last evaluation, for the web UI. Written by loop(), copied out by the web
// task under the lock.
struct View {
    bool    have;
    uint8_t cells;
    uint8_t samples;
    float   meanV;
    float   offMv[MAX_CELLS];
    float   usualMv[MAX_CELLS];   // NAN = not enough comparable readings
    bool    warn[MAX_CELLS];
};
View         s_view;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void driftId(char* out, size_t len, int cell) {
    snprintf(out, len, "M%dC%dDRIFT", cell / 6 + 1, cell % 6 + 1);
}

void raiseWarning(BMSModuleManager& bms, int cell, float dropMv) {
    char id[12], reason[40];
    driftId(id, sizeof(id), cell);
    snprintf(reason, sizeof(reason), "MOD%d C%d FALLING BEHIND %.0fmV", cell / 6 + 1, cell % 6 + 1, dropMv);
    bms.reportFault(id, reason, false, true);   // not blocking, silent
}

void save() {
    Preferences p;
    p.begin("celldrift", false);
    p.putBytes("hist", &s_hist, sizeof(s_hist));
    p.putUShort("bled", s_bledMask);
    p.end();
}

float median(float* v, int n) {
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && v[j] < v[j - 1]; j--) { float t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
    return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

void takeReading(const DisplayData& dd, int cells, BMSModuleManager& bms) {
    float v[MAX_CELLS], sum = 0.0f, lo = 10.0f, hi = 0.0f;
    for (int i = 0; i < cells; i++) {
        v[i] = dd.cellVolt[i / 6][i % 6];
        sum += v[i];
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    float mean = sum / cells;

    Sample now;
    now.meanMv   = (uint16_t)lroundf(mean * 1000.0f);
    now.bledMask = s_bledMask;
    for (int i = 0; i < MAX_CELLS; i++)
        now.off[i] = (i < cells) ? (int16_t)lroundf((v[i] - mean) * 10000.0f) : 0;

    View view;
    view.have = true;
    view.cells = cells;
    view.meanV = mean;

    uint16_t warnMask = 0;
    int judged = 0;
    for (int i = 0; i < cells; i++) {
        // Walk back from the newest earlier reading. Stop at the first interval
        // in which this cell was balanced: anything before that isn't comparable.
        float usual[DRIFT_HISTORY];
        int n = 0;
        uint16_t bledSince = s_bledMask;
        for (int k = 0; k < s_hist.count; k++) {
            if (bledSince & (1u << i)) break;
            const Sample& old = s_hist.s[(s_hist.head + DRIFT_HISTORY - 1 - k) % DRIFT_HISTORY];
            if (abs((int)old.meanMv - (int)now.meanMv) <= (int)lroundf(DRIFT_COMPARE_V * 1000.0f))
                usual[n++] = old.off[i] / 10.0f;
            bledSince |= old.bledMask;
        }

        view.offMv[i]   = now.off[i] / 10.0f;
        view.usualMv[i] = NAN;
        view.warn[i]    = false;
        char id[12];
        driftId(id, sizeof(id), i);
        if (n >= DRIFT_MIN_SAMPLES) {
            judged++;
            float u = median(usual, n);
            view.usualMv[i] = u;
            float drop = u - view.offMv[i];
            if (drop >= DRIFT_WARN_MV) {
                view.warn[i] = true;
                warnMask |= (1u << i);
                s_hist.warnDrop[i] = (int16_t)lroundf(drop * 10.0f);
                raiseWarning(bms, i, drop);
                continue;
            }
        }
        // Not judged this time (too little history, or it was balanced): keep
        // an earlier warning rather than silently dropping it.
        if (n < DRIFT_MIN_SAMPLES && (s_hist.warnMask & (1u << i))) {
            view.warn[i] = true;
            warnMask |= (1u << i);
            continue;
        }
        bms.clearFaultById(id);
    }

    s_hist.s[s_hist.head] = now;
    s_hist.head = (s_hist.head + 1) % DRIFT_HISTORY;
    if (s_hist.count < DRIFT_HISTORY) s_hist.count++;
    s_hist.warnMask = warnMask;
    s_bledMask = 0;
    save();

    view.samples = s_hist.count;
    taskENTER_CRITICAL(&s_mux);
    s_view = view;
    taskEXIT_CRITICAL(&s_mux);

    Blackbox::log("REST", "Rested reading: cells %.3f-%.3f V (spread %.0f mV), %d of %d cells compared with earlier readings",
                  lo, hi, (hi - lo) * 1000.0f, judged, cells);
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

void Celldrift::begin() {
    memset(&s_hist, 0, sizeof(s_hist));
    Preferences p;
    p.begin("celldrift", true);
    if (p.getBytes("hist", &s_hist, sizeof(s_hist)) != sizeof(s_hist))
        memset(&s_hist, 0, sizeof(s_hist));   // nothing saved yet, or saved by a build with a different layout
    s_bledMask = p.getUShort("bled", 0);
    p.end();
    if (s_hist.count > DRIFT_HISTORY || s_hist.head >= DRIFT_HISTORY) memset(&s_hist, 0, sizeof(s_hist));
    s_view.have = false;
}

void Celldrift::poll(const DisplayData& dd, int modulesExpected, int steadyState, bool chargerOn, BMSModuleManager& bms) {
    int cells = (dd.numModules < 2 ? dd.numModules : 2) * 6;

    if (s_resetPending) {
        s_resetPending = false;
        for (int i = 0; i < MAX_CELLS; i++) {
            char id[12];
            driftId(id, sizeof(id), i);
            bms.clearFaultById(id);
        }
        memset(&s_hist, 0, sizeof(s_hist));
        s_bledMask = 0;
        save();
        taskENTER_CRITICAL(&s_mux);
        s_view.have = false;
        taskEXIT_CRITICAL(&s_mux);
        Blackbox::log("NOTE", "Cell drift history reset from the web UI");
    }

    // A warning from an earlier power-on stays visible until a new reading
    // judges that cell again (or the history is reset).
    if (!s_warningsRestored) {
        s_warningsRestored = true;
        for (int i = 0; i < MAX_CELLS; i++)
            if (s_hist.warnMask & (1u << i)) raiseWarning(bms, i, s_hist.warnDrop[i] / 10.0f);
    }

    // Remember every cell that gets balanced, so it isn't judged on a drop the
    // balancing caused. Saved when it changes (a few writes per power-on at most).
    uint16_t bleeding = 0;
    for (int i = 0; i < cells; i++)
        if (dd.cellBalancing[i / 6][i % 6]) bleeding |= (1u << i);
    if (bleeding & ~s_bledMask) {
        s_bledMask |= bleeding;
        Preferences p;
        p.begin("celldrift", false);
        p.putUShort("bled", s_bledMask);
        p.end();
    }

    if (s_done) return;
    // Only a pack at rest from power-on counts: once the lift has moved it or
    // the charger has run, the readings are not rested any more this time.
    if (chargerOn || steadyState == 2 || millis() > DRIFT_SNAPSHOT_WINDOW_MS) { s_done = true; return; }
    if (steadyState != 1) return;
    if (dd.numModules < modulesExpected || cells == 0) return;
    for (int i = 0; i < cells; i++) {
        float v = dd.cellVolt[i / 6][i % 6];
        if (!(v > 1.0f && v < 5.0f)) return;   // a failed read -- wait for a good one
    }
    s_done = true;
    takeReading(dd, cells, bms);
}

void Celldrift::requestReset() { s_resetPending = true; }

String Celldrift::json() {
    View v;
    taskENTER_CRITICAL(&s_mux);
    v = s_view;
    taskEXIT_CRITICAL(&s_mux);

    JsonDocument doc;
    doc["have"]       = v.have;
    doc["stored"]     = s_hist.count;
    doc["history"]    = DRIFT_HISTORY;
    doc["minSamples"] = DRIFT_MIN_SAMPLES;
    doc["warnMv"]     = DRIFT_WARN_MV;
    if (v.have) {
        doc["meanV"] = serialized(String(v.meanV, 3));
        JsonArray cells = doc["cells"].to<JsonArray>();
        for (int i = 0; i < v.cells; i++) {
            JsonObject c = cells.add<JsonObject>();
            c["m"]    = i / 6 + 1;
            c["c"]    = i % 6 + 1;
            c["off"]  = serialized(String(v.offMv[i], 1));
            if (isnan(v.usualMv[i])) c["usual"] = nullptr;
            else                     c["usual"] = serialized(String(v.usualMv[i], 1));
            c["warn"] = v.warn[i];
        }
    }
    String out;
    serializeJson(doc, out);
    return out;
}
