#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Blackbox — a small event log in flash that survives power-off
// ─────────────────────────────────────────────────────────────────────────────
//
//  This board is only powered while the lift runs or the pack charges, and in
//  the field it has no internet and no clock. So everything worth knowing
//  afterwards -- faults, charge sessions, restarts and why they happened,
//  settings changed from the web UI -- is written here, to LittleFS, and read
//  back on the Faults tab of the web UI.
//
//  Each line is   <session>,<seconds since power-on>,<TYPE>,<text>
//  A "session" is one power-on; the counter is kept in NVS. Real dates only
//  exist for a session in which someone opened the web UI: the page sends the
//  phone's clock (no internet needed), which is logged as a CLOCK line, and the
//  page works out the date of every other line in that session from it.
//
//  Flash wear is kept low on purpose: nothing is written periodically except a
//  small NVS checkpoint every 5 minutes, a fault that flickers on and off is
//  logged once rather than every 3 seconds, and each session is capped at
//  BLACKBOX_MAX_LINES_PER_SESSION lines. The log is two files, rotated at boot,
//  so it never grows past about 2 x BLACKBOX_ROTATE_BYTES + one session.
//
//  Threading: log(), setClockFromBrowser(), requestClear() and noteRestart()
//  are safe from any task (the web server's included); they only queue. Every
//  flash write happens in loop(), from Blackbox::loop(). faultRaised(),
//  faultCleared() and chargerTick() are for loop() only.

#define BLACKBOX_ROTATE_BYTES           (40 * 1024)
#define BLACKBOX_MAX_LINES_PER_SESSION  400
#define BLACKBOX_FAULT_SETTLE_MS        30000   // a cleared fault must stay clear this long before its CLEAR is logged
#define BLACKBOX_CHARGER_SETTLE_MS      6000    // the charger's on/off state must hold this long before it counts
#define BLACKBOX_CHECKPOINT_MS          (5UL * 60UL * 1000UL)

#define BLACKBOX_DIR       "/bb"
#define BLACKBOX_LOG       "/bb/log.csv"
#define BLACKBOX_LOG_OLD   "/bb/log_old.csv"

namespace Blackbox {
    void begin();       // setup(): mount LittleFS, count the session, log the boot and how the last session ended
    void loop();        // loop(), every iteration: write whatever is queued

    void log(const char* type, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    // Fault registry hooks (BMSModuleManager::reportFault/clearFaultById).
    void faultRaised(const char* id, const char* reason, bool blocksCharger, bool silent);
    void faultCleared(const char* id, const char* reason, uint32_t startMillis);

    // Called once a second with the charger's actual output state. Logs a
    // CHG_ON / CHG_OFF pair per charge session with its duration, pack voltage
    // before and after, and the amp-hours the charger reported putting in.
    // offReason and cellSpreadMv are only used on the ON -> OFF edge: the spread
    // at the top of a charge shows whether mid-charge balancing (which the pack
    // mostly gets) is leaving the cells apart where it matters.
    void chargerTick(bool outputOn, float amps, float packVolts, float cellSpreadMv, const char* offReason);

    // Say why the next restart happens, just before calling esp_restart(), so
    // the boot line can tell a Reboot button from a firmware update. Kept in
    // RTC memory, which survives a software restart but not a power cut.
    void noteRestart(const char* why);

    void setClockFromBrowser(uint64_t epochMs);   // web task: the viewer's clock
    void requestClear();                          // web task: wipe the log (done in loop())

    uint32_t    session();
    const char* resetReason();     // why this session started, in words
    bool        clockKnown();
}
