#include "Blackbox.h"
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_attr.h>
#include "Logger.h"

// See Blackbox.h for what this is and why it is built the way it is.

namespace {

// ── Queue: filled from any task, emptied (written to flash) by loop() ───────
struct QueuedLine {
    uint32_t upS;
    char     type[8];
    char     text[120];
};
const int QUEUE_LEN = 24;
QueuedLine   s_queue[QUEUE_LEN];
int          s_qHead  = 0;       // next item to write out
int          s_qCount = 0;
uint32_t     s_qDropped = 0;     // lines lost because the queue was full
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

bool     s_mounted = false;
uint32_t s_session = 0;
uint32_t s_linesThisSession = 0;
char     s_resetReason[80] = "unknown";

// Browser clock (set on the web task, logged by loop()).
bool     s_clockKnown   = false;
bool     s_clockPending = false;
uint32_t s_bootEpochS   = 0;     // Unix time at this session's power-on
volatile bool s_clearPending = false;

// Why the next software restart happens. RTC_NOINIT memory keeps its contents
// through esp_restart() but not through a power cut, so the magic number tells
// a real note from leftover garbage.
const uint32_t RESTART_NOTE_MAGIC = 0xB1ACB0C5;
struct RestartNote { uint32_t magic; char why[40]; };
RTC_NOINIT_ATTR RestartNote s_restartNote;

// Faults that cleared but have not stayed clear for BLACKBOX_FAULT_SETTLE_MS
// yet. A fault that comes back inside that window is treated as the same event
// (a flicker), so a cell sitting right on a limit is one FAULT + one CLEAR line
// instead of two lines every 3 seconds. loop() only.
struct PendingClear {
    bool     used;
    bool     active;       // came back while waiting
    char     id[12];
    char     reason[40];
    uint32_t firstStartMs;
    uint32_t clearedAtMs;
    uint16_t flickers;
};
const int PENDING_LEN = 16;
PendingClear s_pending[PENDING_LEN];

// Charge-session tracking (chargerTick(), loop() only).
bool     s_chgOn = false;          // the settled state
bool     s_chgCandidate = false;   // the live state differs from the settled one...
uint32_t s_chgCandidateSinceMs = 0;//   ...since this time
float    s_chgCandidateV = 0.0f;
char     s_chgCandidateReason[40] = "";
float    s_chgCandidateSpreadMv = 0.0f;
uint32_t s_chgStartMs = 0;
float    s_chgStartV = 0.0f;
float    s_chgAh = 0.0f;
float    s_lastPackV = 0.0f;
uint32_t s_lastTickMs = 0;

// Checkpoint in NVS, rewritten every BLACKBOX_CHECKPOINT_MS and on every charge
// start/stop. At the next boot it says how long the last session ran and
// whether it was cut off in the middle of a charge.
struct Checkpoint {
    uint32_t session;
    uint32_t upS;
    uint8_t  charging;
    uint32_t chgSecs;
    float    chgAh;
    float    chgStartV;
    float    packV;
};
uint32_t s_lastCheckpointMs = 0;

void fmtDuration(char* out, size_t len, uint32_t secs) {
    uint32_t h = secs / 3600, m = (secs % 3600) / 60, s = secs % 60;
    if (h)      snprintf(out, len, "%luh %02lum", (unsigned long)h, (unsigned long)m);
    else if (m) snprintf(out, len, "%lum %02lus", (unsigned long)m, (unsigned long)s);
    else        snprintf(out, len, "%lus", (unsigned long)s);
}

void describeReset(char* out, size_t len) {
    const char* why;
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   why = "power switched on"; break;
        case ESP_RST_EXT:       why = "reset pin"; break;
        case ESP_RST_SW:        why = "software restart"; break;
        case ESP_RST_PANIC:     why = "CRASH (software panic)"; break;
        case ESP_RST_INT_WDT:   why = "CRASH (interrupt watchdog)"; break;
        case ESP_RST_TASK_WDT:  why = "HANG (task watchdog: a task stopped for 30 s)"; break;
        case ESP_RST_WDT:       why = "HANG (watchdog)"; break;
        case ESP_RST_DEEPSLEEP: why = "wake from deep sleep"; break;
        case ESP_RST_BROWNOUT:  why = "BROWN-OUT (supply voltage dipped too low)"; break;
        case ESP_RST_SDIO:      why = "SDIO reset"; break;
        default:                why = nullptr; break;
    }
    if (!why) snprintf(out, len, "unknown (code %d)", (int)esp_reset_reason());
    else if (esp_reset_reason() == ESP_RST_SW && s_restartNote.magic == RESTART_NOTE_MAGIC) {
        s_restartNote.why[sizeof(s_restartNote.why) - 1] = '\0';
        snprintf(out, len, "%s: %s", why, s_restartNote.why);
    } else {
        snprintf(out, len, "%s", why);
    }
    s_restartNote.magic = 0;   // used once
}

void writeCheckpoint() {
    Checkpoint ck;
    ck.session   = s_session;
    ck.upS       = millis() / 1000;
    ck.charging  = s_chgOn ? 1 : 0;
    ck.chgSecs   = s_chgOn ? (millis() - s_chgStartMs) / 1000 : 0;
    ck.chgAh     = s_chgOn ? s_chgAh : 0.0f;
    ck.chgStartV = s_chgOn ? s_chgStartV : 0.0f;
    ck.packV     = s_lastPackV;
    Preferences p;
    p.begin("blackbox", false);
    p.putBytes("ckpt", &ck, sizeof(ck));
    p.end();
    s_lastCheckpointMs = millis();
}

void rotateIfLarge() {
    File f = LittleFS.open(BLACKBOX_LOG, FILE_READ);
    if (!f) return;
    size_t size = f.size();
    f.close();
    if (size < BLACKBOX_ROTATE_BYTES) return;
    LittleFS.remove(BLACKBOX_LOG_OLD);
    LittleFS.rename(BLACKBOX_LOG, BLACKBOX_LOG_OLD);
}

bool takeQueued(QueuedLine& out) {
    bool got = false;
    taskENTER_CRITICAL(&s_mux);
    if (s_qCount > 0) {
        out = s_queue[s_qHead];
        s_qHead = (s_qHead + 1) % QUEUE_LEN;
        s_qCount--;
        got = true;
    }
    taskEXIT_CRITICAL(&s_mux);
    return got;
}

void settlePendingClears() {
    uint32_t now = millis();
    for (int i = 0; i < PENDING_LEN; i++) {
        PendingClear& p = s_pending[i];
        if (!p.used || p.active || now - p.clearedAtMs < BLACKBOX_FAULT_SETTLE_MS) continue;
        char dur[16];
        fmtDuration(dur, sizeof(dur), (p.clearedAtMs - p.firstStartMs) / 1000);
        if (p.flickers)
            Blackbox::log("CLEAR", "%s -- over %s, came back %u more time%s",
                          p.reason, dur, (unsigned)p.flickers, p.flickers == 1 ? "" : "s");
        else
            Blackbox::log("CLEAR", "%s -- lasted %s", p.reason, dur);
        p.used = false;
    }
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

void Blackbox::begin() {
    describeReset(s_resetReason, sizeof(s_resetReason));

    Preferences p;
    p.begin("blackbox", false);
    uint32_t prevSession = p.getUInt("session", 0);
    s_session = prevSession + 1;
    p.putUInt("session", s_session);
    Checkpoint ck;
    bool haveCk = (p.getBytes("ckpt", &ck, sizeof(ck)) == sizeof(ck));
    p.end();

    // The partition has never been used before this feature, so the first
    // mount fails and formats it (true). A few seconds, once.
    s_mounted = LittleFS.begin(true);
    if (s_mounted) {
        if (!LittleFS.exists(BLACKBOX_DIR)) LittleFS.mkdir(BLACKBOX_DIR);
        rotateIfLarge();   // only at boot, never while the web UI might be reading the file
    } else {
        Serial.println("Black box: LittleFS failed to mount -- events will not be saved this session");
    }

    log("BOOT", "Power-on #%lu -- started by: %s", (unsigned long)s_session, s_resetReason);

    if (haveCk && ck.session == prevSession) {
        char dur[16];
        fmtDuration(dur, sizeof(dur), ck.upS);
        log("PREV", "Power-on #%lu was still running %s after it started (checked every 5 min)",
            (unsigned long)prevSession, dur);
        if (ck.charging) {
            fmtDuration(dur, sizeof(dur), ck.chgSecs);
            log("CHG_OFF", "Charge cut off by power loss or restart, after at least %s: %.2f V -> %.2f V, %.1f Ah in",
                dur, ck.chgStartV, ck.packV, ck.chgAh);
        }
    }
    writeCheckpoint();   // so a crash within 5 minutes still leaves a record of this session
}

void Blackbox::log(const char* type, const char* fmt, ...) {
    QueuedLine item;
    item.upS = millis() / 1000;
    strncpy(item.type, type, sizeof(item.type) - 1);
    item.type[sizeof(item.type) - 1] = '\0';
    va_list args;
    va_start(args, fmt);
    vsnprintf(item.text, sizeof(item.text), fmt, args);
    va_end(args);
    for (char* c = item.text; *c; c++) if (*c == '\n' || *c == '\r') *c = ' ';   // one event per line

    taskENTER_CRITICAL(&s_mux);
    if (s_qCount < QUEUE_LEN) {
        s_queue[(s_qHead + s_qCount) % QUEUE_LEN] = item;
        s_qCount++;
    } else {
        s_qDropped++;
    }
    taskEXIT_CRITICAL(&s_mux);
}

void Blackbox::loop() {
    settlePendingClears();

    if (s_clockPending) {
        s_clockPending = false;
        log("CLOCK", "%lu = power-on time (Unix), from the clock of a phone/computer viewing the web UI",
            (unsigned long)s_bootEpochS);
    }

    if (s_clearPending) {
        s_clearPending = false;
        if (s_mounted) {
            LittleFS.remove(BLACKBOX_LOG_OLD);
            LittleFS.remove(BLACKBOX_LOG);
        }
        s_linesThisSession = 0;
        log("NOTE", "Black box cleared from the web UI");
    }

    if (millis() - s_lastCheckpointMs >= BLACKBOX_CHECKPOINT_MS) writeCheckpoint();

    if (s_qCount == 0 && s_qDropped == 0) return;   // unlocked peek; a line queued right now goes out next time

    File f;
    if (s_mounted) f = LittleFS.open(BLACKBOX_LOG, FILE_APPEND);

    QueuedLine item;
    while (takeQueued(item)) {
        Logger::info("Black box: %s %s", item.type, item.text);
        if (!f) continue;
        if (s_linesThisSession >= BLACKBOX_MAX_LINES_PER_SESSION) continue;
        s_linesThisSession++;
        if (s_linesThisSession == BLACKBOX_MAX_LINES_PER_SESSION) {
            f.printf("%lu,%lu,NOTE,Line limit for this power-on reached -- later events are not saved\n",
                     (unsigned long)s_session, (unsigned long)item.upS);
            continue;
        }
        f.printf("%lu,%lu,%s,%s\n", (unsigned long)s_session, (unsigned long)item.upS, item.type, item.text);
    }

    uint32_t dropped;
    taskENTER_CRITICAL(&s_mux);
    dropped = s_qDropped;
    s_qDropped = 0;
    taskEXIT_CRITICAL(&s_mux);
    if (dropped && f && s_linesThisSession < BLACKBOX_MAX_LINES_PER_SESSION) {
        s_linesThisSession++;
        f.printf("%lu,%lu,NOTE,%lu events were lost (too many at once)\n",
                 (unsigned long)s_session, (unsigned long)(millis() / 1000), (unsigned long)dropped);
    }

    if (f) f.close();
}

void Blackbox::faultRaised(const char* id, const char* reason, bool blocksCharger, bool silent) {
    for (int i = 0; i < PENDING_LEN; i++) {
        PendingClear& p = s_pending[i];
        if (p.used && strcmp(p.id, id) == 0) {
            if (!p.active) { p.active = true; p.flickers++; }
            return;   // the same event, back again -- no new line
        }
    }
    log(silent ? "WARN" : "FAULT", "%s%s", reason,
        silent ? " (warning, no buzzer)" : blocksCharger ? "" : " (alarm only, charging allowed)");
}

void Blackbox::faultCleared(const char* id, const char* reason, uint32_t startMillis) {
    for (int i = 0; i < PENDING_LEN; i++) {
        PendingClear& p = s_pending[i];
        if (p.used && strcmp(p.id, id) == 0) {
            p.active = false;
            p.clearedAtMs = millis();
            return;
        }
    }
    for (int i = 0; i < PENDING_LEN; i++) {
        PendingClear& p = s_pending[i];
        if (p.used) continue;
        p.used = true;
        p.active = false;
        strncpy(p.id, id, sizeof(p.id) - 1);
        p.id[sizeof(p.id) - 1] = '\0';
        strncpy(p.reason, reason, sizeof(p.reason) - 1);
        p.reason[sizeof(p.reason) - 1] = '\0';
        p.firstStartMs = startMillis;
        p.clearedAtMs = millis();
        p.flickers = 0;
        return;
    }
    // Table full (many faults clearing at once): log it straight away instead.
    char dur[16];
    fmtDuration(dur, sizeof(dur), (millis() - startMillis) / 1000);
    log("CLEAR", "%s -- lasted %s", reason, dur);
}

void Blackbox::chargerTick(bool outputOn, float amps, float packVolts, float cellSpreadMv, const char* offReason) {
    uint32_t now = millis();
    if (s_lastTickMs == 0) s_lastTickMs = now;
    uint32_t dt = now - s_lastTickMs;
    s_lastTickMs = now;
    s_lastPackV = packVolts;

    if (s_chgOn && outputOn && amps > 0.0f) s_chgAh += amps * (float)dt / 3600000.0f;

    if (outputOn == s_chgOn) { s_chgCandidate = false; return; }

    // The state changed. It has to hold for BLACKBOX_CHARGER_SETTLE_MS, so a
    // single missed CAN reply doesn't split one charge into two. The reason and
    // voltage are taken from the moment it first changed.
    if (!s_chgCandidate) {
        s_chgCandidate = true;
        s_chgCandidateSinceMs = now;
        s_chgCandidateV = packVolts;
        s_chgCandidateSpreadMv = cellSpreadMv;
        strncpy(s_chgCandidateReason, offReason ? offReason : "", sizeof(s_chgCandidateReason) - 1);
        s_chgCandidateReason[sizeof(s_chgCandidateReason) - 1] = '\0';
    }
    if (now - s_chgCandidateSinceMs < BLACKBOX_CHARGER_SETTLE_MS) return;

    s_chgCandidate = false;
    s_chgOn = outputOn;
    if (s_chgOn) {
        s_chgStartMs = s_chgCandidateSinceMs;
        s_chgStartV  = s_chgCandidateV;
        s_chgAh      = 0.0f;
        log("CHG_ON", "Charging started, pack %.2f V", s_chgStartV);
    } else {
        char dur[16];
        fmtDuration(dur, sizeof(dur), (s_chgCandidateSinceMs - s_chgStartMs) / 1000);
        log("CHG_OFF", "Charging stopped (%s) after %s: %.2f V -> %.2f V, %.1f Ah in, cell spread %.0f mV",
            s_chgCandidateReason, dur, s_chgStartV, s_chgCandidateV, s_chgAh, s_chgCandidateSpreadMv);
    }
    writeCheckpoint();
}

void Blackbox::noteRestart(const char* why) {
    strncpy(s_restartNote.why, why, sizeof(s_restartNote.why) - 1);
    s_restartNote.why[sizeof(s_restartNote.why) - 1] = '\0';
    s_restartNote.magic = RESTART_NOTE_MAGIC;
    log("NOTE", "Restarting: %s", why);
}

void Blackbox::setClockFromBrowser(uint64_t epochMs) {
    // Anything before 2025 is a phone with no idea of the time; ignore it.
    if (epochMs < 1735689600000ULL) return;
    taskENTER_CRITICAL(&s_mux);
    if (!s_clockKnown) {
        s_bootEpochS   = (uint32_t)(epochMs / 1000ULL) - millis() / 1000;
        s_clockKnown   = true;
        s_clockPending = true;
    }
    taskEXIT_CRITICAL(&s_mux);
}

void Blackbox::requestClear() { s_clearPending = true; }

uint32_t    Blackbox::session()     { return s_session; }
const char* Blackbox::resetReason() { return s_resetReason; }
bool        Blackbox::clockKnown()  { return s_clockKnown; }
