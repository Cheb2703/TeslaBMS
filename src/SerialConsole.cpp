/*
 * SerialConsole.cpp
 *
 Copyright (c) 2017 EVTV / Collin Kidder

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include "SerialConsole.h"
#include "Logger.h"
#include "BMSModuleManager.h"
#include "ChargerNPB.h"
#include <Preferences.h>

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Lets us stream SerialUSB

extern EEPROMSettings settings;
extern BMSModuleManager bms;
extern bool testFaultOverride;
extern uint32_t testFaultUntilMillis;
extern Preferences preferences;

// Charger setpoints, persisted the same way as balanceVoltage/balanceHyst.
// Defined in main.cpp -- these are what printMenu() displays live and what
// the CHGxxx commands below update/save.
extern float    chargerVoltage;
extern float    chargerCurrent;
extern float    chargerCurveCC;
extern float    chargerCurveCV;
extern float    chargerCurveFV;
extern float    chargerCurveTC;
extern float    chargerRstVbat;
extern uint16_t chargerCCTimeoutMin;
extern uint16_t chargerCVTimeoutMin;
extern uint16_t chargerFVTimeoutMin;
extern bool     desiredChargerOn;   // what we WANT the charger's output to be -- see main.cpp
extern bool     currentFaultState;  // true while a BMS fault is active -- see main.cpp
extern bool     chargerReady;       // true once charger.begin() succeeded at boot -- see main.cpp
extern float    chargerDailyTargetV;
extern float    chargerFullTargetV;
extern bool     chargerFullChargeOverride;
extern void     applyChargeTargetVoltage();

// Shared by every CHGxxx curve-parameter command below. Curve-family
// registers (CURVE_CC/CV/FV/TC, CHG_RST_VBAT, the *_TIMEOUT registers) only
// take effect on a remote/comm on-off toggle or AC power cycle per the
// manual -- these helpers force that toggle right now via
// ChargerNPB::reapplyCurveNow(), but only when the charger is actually
// supposed to be running (desiredChargerOn) and no BMS fault is holding it
// off. Otherwise the value is saved/written and will simply take effect the
// next time the output is turned on -- no point briefly toggling a charger
// that's supposed to be off anyway.
static void reapplyIfRunning(const char* label, float oldVal, float newVal) {
    if (chargerReady && desiredChargerOn && !currentFaultState) {
        if (charger.reapplyCurveNow())
            Logger::console("%s: was %f, now %f -- reapplied live (charger briefly toggled off/on)", label, oldVal, newVal);
        else
            Logger::console("%s: saved %f but live reapply failed to confirm -- charger not responding", label, newVal);
    } else {
        Logger::console("%s: was %f, now %f -- saved, will take effect next time output is turned on", label, oldVal, newVal);
    }
}

static void reapplyIfRunningInt(const char* label, uint16_t oldVal, uint16_t newVal) {
    if (chargerReady && desiredChargerOn && !currentFaultState) {
        if (charger.reapplyCurveNow())
            Logger::console("%s: was %d, now %d -- reapplied live (charger briefly toggled off/on)", label, oldVal, newVal);
        else
            Logger::console("%s: saved %d but live reapply failed to confirm -- charger not responding", label, newVal);
    } else {
        Logger::console("%s: was %d, now %d -- saved, will take effect next time output is turned on", label, oldVal, newVal);
    }
}

bool printPrettyDisplay;
uint32_t prettyCounter;
//int whichDisplay;

bool printDisplay;      //true = print
unsigned long displayPreviousMillis=0;

#define NONE 0      //Constants to define which display is sent to the serial monitor
#define SUMMARY 1
#define DETAILS 2
#define JSON 3
byte whichDisplay;     //the variable to hold them

SerialConsole::SerialConsole() {
    init();
}

void SerialConsole::init() {
    //State variables for serial console
    ptrBuffer = 0;
    state = STATE_ROOT_MENU;
    //loopcount=0;
    //cancel=false;
    printDisplay = false;
    prettyCounter = 0;
    whichDisplay = NONE;
}

void SerialConsole::loop() {
    unsigned long currentMillis = millis();
    unsigned long interval = 3000;
    if (SERIALCONSOLE.available()) {
        serialEvent();
    }
    if (printDisplay && ((currentMillis - displayPreviousMillis) >= interval))
    {
        displayPreviousMillis = currentMillis;
        if (whichDisplay == SUMMARY) bms.printPackSummary();
        if (whichDisplay == DETAILS) bms.printPackDetails();
        //if (whichDisplay == JSON) bms.jsonData();
    }
}

void SerialConsole::printMenu() {
    Logger::console("\n*************SYSTEM MENU *****************");
    Logger::console("Enable line endings of some sort (LF, CR, CRLF)");
    Logger::console("Most commands case sensitive\n");
    Logger::console("GENERAL SYSTEM CONFIGURATION\n");
    Logger::console("   E = dump system EEPROM values");
    Logger::console("   h = help (displays this message)");
    Logger::console("   S = Sleep all boards");
    Logger::console("   W = Wake up all boards");
    Logger::console("   C = Clear all board faults");
    Logger::console("   F = Find all connected boards");
    Logger::console("   R = Renumber connected boards in sequence");
    Logger::console("   B = Manual start balancing");
    Logger::console("   b = Manual stop balancing");
    Logger::console("   1 to 6 = Toggle balancing on cell 1 to 6");
    Logger::console("   t = Inject a 5-second test fault (verify buzzer/display/history)");
    Logger::console("   o = Toggle charger output ON/OFF");
    Logger::console("   y = Print charger status now");
    Logger::console("   u = Toggle full-charge override");
    Logger::console("   p = Toggle output of pack summary every 3 seconds");
    Logger::console("   d = Toggle output of pack details every 3 seconds");
    Logger::console("   j = display JSON Data every 3 seconds, toggle off");

    Logger::console("   LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", Logger::getLogLevel());

    Logger::console("\nBATTERY MANAGEMENT CONTROLS\n");
    Logger::console("   VOLTLIMHI=%f - High limit for cells in volts", settings.OverVSetpoint);
    Logger::console("   VOLTLIMLO=%f - Low limit for cells in volts", settings.UnderVSetpoint);
    Logger::console("   TEMPLIMHI=%f - High limit for cell temperature in degrees C", settings.OverTSetpoint);
    Logger::console("   TEMPLIMLO=%f - Low limit for cell temperature in degrees C", settings.UnderTSetpoint);
    Logger::console("   BALVOLT=%f - Voltage at which to begin cell balancing", settings.balanceVoltage);
    Logger::console("   BALHYST=%f - How far voltage must dip before balancing is turned off", settings.balanceHyst);

    Logger::console("\nCHARGER CONTROLS (MEAN WELL NPB-750-24, CANBus)\n");
    Logger::console("   o = Toggle charger output ON/OFF");
    Logger::console("   y = Print charger status now (measured values, faults)");
    Logger::console("   u = Toggle full-charge override (daily-limit charging otherwise)");
    Logger::console("   CHGDAILYV=%f - Everyday charge target, ~80%% SOC (21.0-42.0V)", chargerDailyTargetV);
    Logger::console("   CHGFULLV=%f  - Full-charge override target (21.0-42.0V)", chargerFullTargetV);
    Logger::console("   CHGV=%f    - Charging voltage target (alias for CHGCV, 21.0-42.0V)", chargerVoltage);
    Logger::console("   CHGI=%f    - Charging current target (alias for CHGCC, 0-22.5A)", chargerCurrent);
    Logger::console("   CHGCC=%f   - Curve constant-current target (0-22.5A)*", chargerCurveCC);
    Logger::console("   CHGCV=%f   - Curve constant-voltage target (21.0-42.0V)*", chargerCurveCV);
    Logger::console("   CHGFV=%f   - Curve float-voltage target (21.0-42.0V)*", chargerCurveFV);
    Logger::console("   CHGTC=%f   - Curve taper-current cutoff (0-22.5A)*", chargerCurveTC);
    Logger::console("   CHGRSTV=%f - Auto-restart-charge voltage point (21.0-42.0V)*", chargerRstVbat);
    Logger::console("   CHGCCTO=%d - CC-stage timeout in minutes, 0=disabled*", chargerCCTimeoutMin);
    Logger::console("   CHGCVTO=%d - CV-stage timeout in minutes, 0=disabled*", chargerCVTimeoutMin);
    Logger::console("   CHGFVTO=%d - Float-stage timeout in minutes, 0=disabled*", chargerFVTimeoutMin);
    Logger::console("   * all curve-related settings are always saved immediately, and reapplied");
    Logger::console("     live (via a brief confirmed off/on toggle) if the charger is currently");
    Logger::console("     commanded on. If it's off (or a BMS fault is active), the value just");
    Logger::console("     takes effect the next time output is turned on -- no interruption now.");
    Logger::console("   CHGRAWW=0xB4,0x0004 - Advanced: write any charger register directly (cmd,value)");
    Logger::console("   CHGRAWR=0xB4         - Advanced: read any charger register directly");
    Logger::console("   CHGOPINIT=0 - SYSTEM_CONFIG power-on behavior (0=OFF/recommended, 1=ON, 2=last state)");
    Logger::console("     Only touches the OPERATION_INIT bits, leaves RSTE/EEP_OFF/etc. untouched.");
    Logger::console("     Takes effect on the charger's NEXT AC power-up, not live.");

    Logger::console("   \nz = Restart the Board");

    float OverVSetpoint;
    float UnderVSetpoint;
    float OverTSetpoint;
    float UnderTSetpoint;
    float balanceVoltage;
    float balanceHyst;
}

/*	There is a help menu (press H or h or ?)

    Commands are submitted by sending line ending (LF, CR, or both)
 */
void SerialConsole::serialEvent() {
    int incoming;
    incoming = SERIALCONSOLE.read();
    if (incoming == -1) { //false alarm....
        return;
    }

    // Echo the character straight back so it's visible while typing -- PuTTY
    // and the PlatformIO/CLion serial monitor don't do local echo by default,
    // so without this nothing you type shows up on screen until you've
    // already sent the whole line (and often not even then).
    SERIALCONSOLE.write((uint8_t)incoming);

    if (incoming == 10 || incoming == 13) { //command done. Parse it.
        handleConsoleCmd();
        ptrBuffer = 0; //reset line counter once the line has been processed
    } else {
        cmdBuffer[ptrBuffer++] = (unsigned char) incoming;
        if (ptrBuffer > 79)
            ptrBuffer = 79;
    }
}

void SerialConsole::handleConsoleCmd() {

    if (state == STATE_ROOT_MENU) {
        if (ptrBuffer == 1) { //command is a single ascii character
            handleShortCmd();
        } else { //if cmd over 1 char then assume (for now) that it is a config line
            handleConfigCmd();
        }
    }
}

// Web-console entry point -- loads cmdBuffer/ptrBuffer exactly like
// serialEvent() would have after receiving a line ending, then reuses
// handleConsoleCmd() unchanged. This is the ONLY place besides serialEvent()
// that touches cmdBuffer/ptrBuffer, so short commands (o, y, t, 1-6, etc.)
// and NAME=value config lines both work identically from the web tab.
void SerialConsole::injectLine(const String& line) {
    int n = line.length();
    if (n > 79) n = 79;
    for (int i = 0; i < n; i++) {
        cmdBuffer[i] = (unsigned char)line[i];
    }
    ptrBuffer = n;
    if (ptrBuffer > 0) {
        handleConsoleCmd();
    }
    ptrBuffer = 0;
}

/*For simplicity the configuration setting code uses four characters for each configuration choice. This makes things easier for
 comparison purposes.
 */
void SerialConsole::handleConfigCmd() {
    int i;
    int newValue;
    float newFloat;

    //Logger::debug("Cmd size: %i", ptrBuffer);
    if (ptrBuffer < 6)
        return; //4 digit command, =, value is at least 6 characters
    cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
    String cmdString = String();
    unsigned char whichEntry = '0';
    i = 0;

    while (cmdBuffer[i] != '=' && i < ptrBuffer) {
        cmdString.concat(String(cmdBuffer[i++]));
    }
    i++; //skip the =
    if (i >= ptrBuffer)
    {
        Logger::console("Command needs a value..ie TORQ=3000");
        Logger::console("");
        return; //or, we could use this to display the parameter instead of setting
    }

    // strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
    newValue = strtol((char *) (cmdBuffer + i), NULL, 0);
    newFloat = strtof((char *) (cmdBuffer + i), NULL);

    cmdString.toUpperCase();

    // Every branch below persists its setting immediately via Preferences
    // (NVS) rather than RAM-only, so it survives a reboot with no WiFi/WebUI
    // needed -- this mirrors the same storage the WebUI already uses for
    // balanceVoltage/balanceHyst, just extended to the rest of the settings.
    if (cmdString == String("CANSPEED")) {
        if (newValue >= 33000 && newValue <= 1000000) {
            uint32_t oldVal = settings.canSpeed;
            settings.canSpeed = newValue;
            preferences.begin("settings", false);
            preferences.putUInt("canSpeed", settings.canSpeed);
            preferences.end();
            Logger::console("CANSPEED: was %l, now %l", oldVal, settings.canSpeed);
        }
        else Logger::console("Invalid speed. Enter a value between 33000 and 1000000");
    } else if (cmdString == String("LOGLEVEL")) {
        uint8_t oldVal = settings.logLevel;
        switch (newValue) {
        case 0:
            Logger::setLoglevel(Logger::Debug);
            settings.logLevel = 0;
            break;
        case 1:
            Logger::setLoglevel(Logger::Info);
            settings.logLevel = 1;
            break;
        case 2:
            settings.logLevel = 2;
            Logger::setLoglevel(Logger::Warn);
            break;
        case 3:
            settings.logLevel = 3;
            Logger::setLoglevel(Logger::Error);
            break;
        case 4:
            settings.logLevel = 4;
            Logger::setLoglevel(Logger::Off);
            break;
        default:
            Logger::console("Invalid log level. Enter a value 0-4");
            return; // don't persist an unrecognized value
        }
        preferences.begin("settings", false);
        preferences.putUChar("logLevel", settings.logLevel);
        preferences.end();
        Logger::console("LOGLEVEL: was %d, now %d", oldVal, settings.logLevel);
    } else if (cmdString == String("BATTERYID")) {
        if (newValue > 0 && newValue < 15) {
            uint8_t oldVal = settings.batteryID;
            settings.batteryID = newValue;
            preferences.begin("settings", false);
            preferences.putUChar("batteryID", settings.batteryID);
            preferences.end();
            //bms.setBatteryID();
            Logger::console("BATTERYID: was %d, now %d", oldVal, settings.batteryID);
        }
        else Logger::console("Invalid battery ID. Please enter a value between 1 and 14");
    } else if (cmdString == String("VOLTLIMHI")) {
        if (newFloat >= 0.0f && newFloat <= 6.00f) {
            float oldVal = settings.OverVSetpoint;
            settings.OverVSetpoint = newFloat;
            preferences.begin("settings", false);
            preferences.putFloat("overVSetpoint", settings.OverVSetpoint);
            preferences.end();
            Logger::console("VOLTLIMHI: was %f, now %f", oldVal, settings.OverVSetpoint);
        }
        else Logger::console("Invalid upper cell voltage limit. Please enter a value 0.0 to 6.0");
    } else if (cmdString == String("VOLTLIMLO")) {
        if (newFloat >= 0.0f && newFloat <= 6.0f) {
            float oldVal = settings.UnderVSetpoint;
            settings.UnderVSetpoint = newFloat;
            preferences.begin("settings", false);
            preferences.putFloat("underVSetpoint", settings.UnderVSetpoint);
            preferences.end();
            Logger::console("VOLTLIMLO: was %f, now %f", oldVal, settings.UnderVSetpoint);
        }
        else Logger::console("Invalid lower cell voltage limit. Please enter a value 0.0 to 6.0");
    } else if (cmdString == String("BALVOLT")) {
        if (newFloat >= 0.0f && newFloat <= 6.0f) {
            float oldVal = settings.balanceVoltage;
            settings.balanceVoltage = newFloat;
            preferences.begin("settings", false);
            preferences.putFloat("balanceVoltage", settings.balanceVoltage);
            preferences.end();
            Logger::console("BALVOLT: was %f, now %f", oldVal, settings.balanceVoltage);
        }
        else Logger::console("Invalid balancing voltage. Please enter a value 0.0 to 6.0");
    } else if (cmdString == String("BALHYST")) {
        if (newFloat >= 0.0f && newFloat <= 1.0f) {
            float oldVal = settings.balanceHyst;
            settings.balanceHyst = newFloat;
            preferences.begin("settings", false);
            preferences.putFloat("balanceHyst", settings.balanceHyst);
            preferences.end();
            Logger::console("BALHYST: was %f, now %f", oldVal, settings.balanceHyst);
        }
        else Logger::console("Invalid balance hysteresis. Please enter a value 0.0 to 1.0");
    } else if (cmdString == String("TEMPLIMHI")) {
        if (newFloat >= 0.0f && newFloat <= 100.0f) {
            float oldVal = settings.OverTSetpoint;
            settings.OverTSetpoint = newFloat;
            preferences.begin("settings", false);
            preferences.putFloat("overTSetpoint", settings.OverTSetpoint);
            preferences.end();
            Logger::console("TEMPLIMHI: was %f, now %f", oldVal, settings.OverTSetpoint);
        }
        else Logger::console("Invalid temperature upper limit please enter a value 0.0 to 100.0");
    } else if (cmdString == String("TEMPLIMLO")) {
        if (newFloat >= -20.00f && newFloat <= 120.0f) {
            float oldVal = settings.UnderTSetpoint;
            settings.UnderTSetpoint = newFloat;
            preferences.begin("settings", false);
            preferences.putFloat("underTSetpoint", settings.UnderTSetpoint);
            preferences.end();
            Logger::console("TEMPLIMLO: was %f, now %f", oldVal, settings.UnderTSetpoint);
        }
        else Logger::console("Invalid temperature lower limit please enter a value between -20.0 and 120.0");

    // ── Charger (MEAN WELL NPB-750-24, CANBus) ────────────────────────────
    // Every branch below both pushes the write over CAN immediately AND
    // persists it to Preferences + updates the global, same pattern as
    // VOLTLIMHI/BALVOLT above -- so printMenu() always shows the true
    // current value and it survives an ESP32 reboot.
    //
    // IMPORTANT: the charger only ever runs its curve/battery-charging
    // profile in this application (never bare PSU/direct-output mode), and
    // VOUT_SET/IOUT_SET are IGNORED in that mode -- CURVE_CV/CURVE_CC are
    // what actually govern output voltage/current. So CHGV/CHGI below write
    // CURVE_CV/CURVE_CC (mirrored into chargerCurveCV/chargerCurveCC so
    // "voltage"/"current" and "curve voltage"/"curve current" always agree),
    // and every CHGxxx curve command reapplies live via a brief confirmed
    // off/on toggle IF the charger is currently supposed to be running --
    // see reapplyIfRunning()/reapplyIfRunningInt() above. If it's not
    // currently running (or a fault is active), the value is just saved and
    // takes effect the next time output is turned on.
    } else if (cmdString == String("CHGV")) {
        float oldVal = chargerVoltage;
        chargerVoltage = newFloat < NPB24_VOLT_MIN ? NPB24_VOLT_MIN : (newFloat > NPB24_VOLT_MAX ? NPB24_VOLT_MAX : newFloat);
        chargerCurveCV = chargerVoltage; // same physical quantity -- CURVE_CV is what actually governs it
        preferences.begin("settings", false);
        preferences.putFloat("chgVoltage", chargerVoltage);
        preferences.putFloat("chgCurveCV", chargerCurveCV);
        preferences.end();
        if (charger.setCurveCV(chargerCurveCV))
            reapplyIfRunning("CHGV", oldVal, chargerVoltage);
        else
            Logger::console("CHGV: saved %f but write to charger failed -- charger not responding", chargerVoltage);
    } else if (cmdString == String("CHGI")) {
        float oldVal = chargerCurrent;
        chargerCurrent = newFloat < NPB24_CURR_MIN ? NPB24_CURR_MIN : (newFloat > NPB24_CURR_MAX ? NPB24_CURR_MAX : newFloat);
        chargerCurveCC = chargerCurrent; // same physical quantity -- CURVE_CC is what actually governs it
        preferences.begin("settings", false);
        preferences.putFloat("chgCurrent", chargerCurrent);
        preferences.putFloat("chgCurveCC", chargerCurveCC);
        preferences.end();
        if (charger.setCurveCC(chargerCurveCC))
            reapplyIfRunning("CHGI", oldVal, chargerCurrent);
        else
            Logger::console("CHGI: saved %f but write to charger failed -- charger not responding", chargerCurrent);
    } else if (cmdString == String("CHGCC")) {
        float oldVal = chargerCurveCC;
        chargerCurveCC = newFloat < NPB24_CURR_MIN ? NPB24_CURR_MIN : (newFloat > NPB24_CURR_MAX ? NPB24_CURR_MAX : newFloat);
        chargerCurrent = chargerCurveCC; // keep the CHGI/"live current" view in sync
        preferences.begin("settings", false);
        preferences.putFloat("chgCurveCC", chargerCurveCC);
        preferences.putFloat("chgCurrent", chargerCurrent);
        preferences.end();
        if (charger.setCurveCC(chargerCurveCC))
            reapplyIfRunning("CHGCC", oldVal, chargerCurveCC);
        else
            Logger::console("CHGCC: saved %f but write to charger failed -- charger not responding", chargerCurveCC);
    } else if (cmdString == String("CHGDAILYV")) {
        float oldVal = chargerDailyTargetV;
        chargerDailyTargetV = newFloat < NPB24_VOLT_MIN ? NPB24_VOLT_MIN : (newFloat > NPB24_VOLT_MAX ? NPB24_VOLT_MAX : newFloat);
        preferences.begin("settings", false);
        preferences.putFloat("chgDailyV", chargerDailyTargetV);
        preferences.end();
        applyChargeTargetVoltage();
        if (!chargerFullChargeOverride) {
            reapplyIfRunning("CHGDAILYV", oldVal, chargerDailyTargetV);
        } else {
            Logger::console("CHGDAILYV: was %f, now %f -- saved (currently in FULL CHARGE OVERRIDE, won't apply until that ends)", oldVal, chargerDailyTargetV);
        }
    } else if (cmdString == String("CHGFULLV")) {
        float oldVal = chargerFullTargetV;
        chargerFullTargetV = newFloat < NPB24_VOLT_MIN ? NPB24_VOLT_MIN : (newFloat > NPB24_VOLT_MAX ? NPB24_VOLT_MAX : newFloat);
        preferences.begin("settings", false);
        preferences.putFloat("chgFullV", chargerFullTargetV);
        preferences.end();
        applyChargeTargetVoltage();
        if (chargerFullChargeOverride) {
            reapplyIfRunning("CHGFULLV", oldVal, chargerFullTargetV);
        } else {
            Logger::console("CHGFULLV: was %f, now %f -- saved (not currently in full-charge override)", oldVal, chargerFullTargetV);
        }
    } else if (cmdString == String("CHGCV")) {
        // Manual/ad-hoc override -- gets overwritten the next time the daily/
        // full-charge target system re-asserts itself (boot, 'u' toggle, or
        // full-charge auto-completion). Use CHGDAILYV/CHGFULLV to change the
        // actual targets that system uses.
        float oldVal = chargerCurveCV;
        chargerCurveCV = newFloat < NPB24_VOLT_MIN ? NPB24_VOLT_MIN : (newFloat > NPB24_VOLT_MAX ? NPB24_VOLT_MAX : newFloat);
        chargerVoltage = chargerCurveCV; // keep the CHGV/"live voltage" view in sync
        preferences.begin("settings", false);
        preferences.putFloat("chgCurveCV", chargerCurveCV);
        preferences.putFloat("chgVoltage", chargerVoltage);
        preferences.end();
        if (charger.setCurveCV(chargerCurveCV))
            reapplyIfRunning("CHGCV", oldVal, chargerCurveCV);
        else
            Logger::console("CHGCV: saved %f but write to charger failed -- charger not responding", chargerCurveCV);
    } else if (cmdString == String("CHGFV")) {
        // Same caveat as CHGCV above -- gets overwritten by applyChargeTargetVoltage().
        float oldVal = chargerCurveFV;
        chargerCurveFV = newFloat < NPB24_VOLT_MIN ? NPB24_VOLT_MIN : (newFloat > NPB24_VOLT_MAX ? NPB24_VOLT_MAX : newFloat);
        preferences.begin("settings", false);
        preferences.putFloat("chgCurveFV", chargerCurveFV);
        preferences.end();
        if (charger.setCurveFV(chargerCurveFV))
            reapplyIfRunning("CHGFV", oldVal, chargerCurveFV);
        else
            Logger::console("CHGFV: saved %f but write to charger failed -- charger not responding", chargerCurveFV);
    } else if (cmdString == String("CHGTC")) {
        float oldVal = chargerCurveTC;
        chargerCurveTC = newFloat < NPB24_CURR_MIN ? NPB24_CURR_MIN : (newFloat > NPB24_CURR_MAX ? NPB24_CURR_MAX : newFloat);
        preferences.begin("settings", false);
        preferences.putFloat("chgCurveTC", chargerCurveTC);
        preferences.end();
        if (charger.setCurveTC(chargerCurveTC))
            reapplyIfRunning("CHGTC", oldVal, chargerCurveTC);
        else
            Logger::console("CHGTC: saved %f but write to charger failed -- charger not responding", chargerCurveTC);
    } else if (cmdString == String("CHGRSTV")) {
        float oldVal = chargerRstVbat;
        chargerRstVbat = newFloat < NPB24_VOLT_MIN ? NPB24_VOLT_MIN : (newFloat > NPB24_VOLT_MAX ? NPB24_VOLT_MAX : newFloat);
        preferences.begin("settings", false);
        preferences.putFloat("chgRstVbat", chargerRstVbat);
        preferences.end();
        if (charger.setChgRstVbat(chargerRstVbat))
            reapplyIfRunning("CHGRSTV", oldVal, chargerRstVbat);
        else
            Logger::console("CHGRSTV: saved %f but write to charger failed -- charger not responding", chargerRstVbat);
    } else if (cmdString == String("CHGCCTO")) {
        if (newValue >= 0 && newValue <= 6000) {
            uint16_t oldVal = chargerCCTimeoutMin;
            chargerCCTimeoutMin = (uint16_t)newValue;
            preferences.begin("settings", false);
            preferences.putUShort("chgCCTmout", chargerCCTimeoutMin);
            preferences.end();
            if (charger.setCurveCCTimeoutMinutes(chargerCCTimeoutMin))
                reapplyIfRunningInt("CHGCCTO", oldVal, chargerCCTimeoutMin);
            else
                Logger::console("CHGCCTO: saved %d but write to charger failed -- charger not responding", chargerCCTimeoutMin);
        } else Logger::console("Invalid CC timeout. Enter 0-6000 minutes (0 disables).");
    } else if (cmdString == String("CHGCVTO")) {
        if (newValue >= 0 && newValue <= 6000) {
            uint16_t oldVal = chargerCVTimeoutMin;
            chargerCVTimeoutMin = (uint16_t)newValue;
            preferences.begin("settings", false);
            preferences.putUShort("chgCVTmout", chargerCVTimeoutMin);
            preferences.end();
            if (charger.setCurveCVTimeoutMinutes(chargerCVTimeoutMin))
                reapplyIfRunningInt("CHGCVTO", oldVal, chargerCVTimeoutMin);
            else
                Logger::console("CHGCVTO: saved %d but write to charger failed -- charger not responding", chargerCVTimeoutMin);
        } else Logger::console("Invalid CV timeout. Enter 0-6000 minutes (0 disables).");
    } else if (cmdString == String("CHGFVTO")) {
        if (newValue >= 0 && newValue <= 6000) {
            uint16_t oldVal = chargerFVTimeoutMin;
            chargerFVTimeoutMin = (uint16_t)newValue;
            preferences.begin("settings", false);
            preferences.putUShort("chgFVTmout", chargerFVTimeoutMin);
            preferences.end();
            if (charger.setCurveFVTimeoutMinutes(chargerFVTimeoutMin))
                reapplyIfRunningInt("CHGFVTO", oldVal, chargerFVTimeoutMin);
            else
                Logger::console("CHGFVTO: saved %d but write to charger failed -- charger not responding", chargerFVTimeoutMin);
        } else Logger::console("Invalid float timeout. Enter 0-6000 minutes (0 disables).");
    } else if (cmdString == String("CHGRAWW")) {
        // Advanced: CHGRAWW=<cmd>,<value>  e.g. CHGRAWW=0xB4,0x0004
        // Both fields accept hex (0x..) or decimal via strtol's base-0 parsing.
        // Not persisted/tracked as a named setting -- this writes arbitrary
        // registers, so there's no single friendly value to redisplay in h.
        char* commaPos = strchr((char*)(cmdBuffer + i), ',');
        if (!commaPos) {
            Logger::console("CHGRAWW needs two values separated by a comma: CHGRAWW=<cmd>,<value>  e.g. CHGRAWW=0xB4,0x0004");
        } else {
            uint16_t rawCmd = (uint16_t)strtol((char*)(cmdBuffer + i), NULL, 0);
            uint16_t rawVal = (uint16_t)strtol(commaPos + 1, NULL, 0);
            if (charger.writeRaw(rawCmd, rawVal))
                Logger::console("CHGRAWW: wrote 0x%X = 0x%X directly. Verify against the manual's bit tables for that register.", rawCmd, rawVal);
            else
                Logger::console("CHGRAWW: write failed -- charger not responding");
        }
    } else if (cmdString == String("CHGRAWR")) {
        // Advanced: CHGRAWR=<cmd>  e.g. CHGRAWR=0xB4
        uint16_t rawCmd = (uint16_t)newValue;
        uint16_t rawVal;
        if (charger.readRaw(rawCmd, rawVal))
            Logger::console("CHGRAWR: register 0x%X = 0x%X (%d decimal)", rawCmd, rawVal, rawVal);
        else
            Logger::console("CHGRAWR: no reply for register 0x%X", rawCmd);

    } else if (cmdString == String("CHGOPINIT")) {
        // Sets ONLY the OPERATION_INIT bits (low byte, bits 1-2) of
        // SYSTEM_CONFIG (0x00C2) -- this is what decides whether the
        // charger's DC output comes up on its own at AC power-on, with NO
        // CAN traffic involved at all:
        //   0 = power on with output OFF (needs an OPERATION=ON command
        //       from the ESP32 before it'll ever charge -- recommended,
        //       since it means "no BMS, no charge" even across an AC
        //       power-cycle on the charger itself)
        //   1 = power on with output ON, unconditionally
        //   2 = power on with whatever OPERATION state was last commanded
        //   3 = unused per the manual
        // Reads the current raw register first and only rewrites bits 1-2,
        // so RSTE/EEP_OFF/CAN_CTRL/etc. are left exactly as they were --
        // unlike CHGRAWW, which would clobber the whole register if you
        // didn't reconstruct every other bit by hand.
        // NOTE: like the other *_CONFIG bits, this takes effect on the
        // charger's NEXT AC power-up, not live.
        if (newValue < 0 || newValue > 2) {
            Logger::console("Invalid CHGOPINIT value. Use 0=power-on OFF (recommended), 1=power-on ON, 2=power-on with last state.");
        } else {
            uint16_t raw;
            if (!charger.readRaw(NPB_SYSTEM_CONFIG, raw)) {
                Logger::console("CHGOPINIT: couldn't read current SYSTEM_CONFIG -- charger not responding");
            } else {
                uint16_t newRaw = (uint16_t)((raw & ~(uint16_t)0x0006) | ((uint16_t)newValue << 1));
                if (charger.writeRaw(NPB_SYSTEM_CONFIG, newRaw)) {
                    static const char* desc[3] = { "power-on OFF", "power-on ON", "power-on with last commanded state" };
                    Logger::console("CHGOPINIT: SYSTEM_CONFIG was %X, now %X (OPERATION_INIT=%d, %s). Takes effect on the charger's NEXT AC power-up.",
                                     raw, newRaw, newValue, desc[newValue]);
                } else {
                    Logger::console("CHGOPINIT: write failed -- charger not responding");
                }
            }
        }
    } else {
        Logger::console("Unknown command");
    }
}

void SerialConsole::handleShortCmd() {
    uint8_t val;

    switch (cmdBuffer[0]) {
    case 'h': case '?': case 'H':
        printMenu();
        break;
    case 's': case 'S':
        Logger::console("Sleeping all connected boards");
        bms.sleepBoards();
        break;
    case 'w': case 'W':
        Logger::console("Waking up all connected boards");
        bms.wakeBoards();
        break;
    case 'c': case 'C':
        Logger::console("Clearing all faults");
        bms.clearFaults();
        break;
    case 'f': case 'F':
        Logger::console("Finding boards.");
        bms.findBoards();
        break;
    case 'r': case 'R':
        Logger::console("Renumbering all boards.");
        bms.renumberBoardIDs();
        break;
    case 'b': case 'B':
        bms.balanceCells();
        break;
    case 'z':
        Logger::console("Restart commanded");
        esp_restart();
        break;
    case '1': case '2': case '3': case '4': case '5': case '6':
        bms.balanceCell(cmdBuffer[0] - '0'); // Basically removing 48 to convert from ascii to int
        break;
    case 't': case 'T':
        testFaultOverride = true;
        testFaultUntilMillis = millis() + 5000;
        Logger::console("Injecting a 5-second test fault -- buzzer should chirp, display should show FAULT, and fault history should update once it clears.");
        break;
    case 'o': case 'O': {
        bool newState = !charger.data().outputOn;
        desiredChargerOn = newState; // this is what we want going forward -- the
                                      // background mismatch-correction in main.cpp
                                      // loop() will keep enforcing this even if the
                                      // charger drifts (e.g. auto-restart after an
                                      // AC power cycle).
        bool confirmed;
        if (charger.setOutputConfirmed(newState, confirmed))
            Logger::console("Charger output %s (confirmed)", confirmed ? "ON" : "OFF");
        else
            Logger::console("Charger output command sent but NOT confirmed within timeout -- desired state saved, background correction will keep retrying.");
        break;
    }
    case 'y': case 'Y': {
        charger.poll();
        const ChargerData& cd = charger.data();
        Logger::console("Charger status: %s", cd.online ? "ONLINE" : "OFFLINE (no reply)");
        if (cd.online) {
            Logger::console("  MEASURED: VOUT=%fV  IOUT=%fA  TEMP=%fC  outputOn=%d", cd.vout, cd.iout, cd.temp, cd.outputOn);
            Logger::console("  FAULT_STATUS=0x%X (%s)  CHG_STATUS=0x%X (%s)", cd.faultRaw, ChargerNPB::faultToString(cd.faultRaw).c_str(), cd.chgStatus, ChargerNPB::chgStatusToString(cd.chgStatus).c_str());
        }
        uint16_t sysCfg;
        if (charger.readRaw(NPB_SYSTEM_CONFIG, sysCfg)) {
            uint8_t opInit = (sysCfg >> 1) & 0x03;
            static const char* opInitDesc[4] = { "power-on OFF", "power-on ON", "power-on with last commanded state", "unused" };
            bool rste   = (sysCfg & (1 << 3))  != 0;
            bool eepOff = (sysCfg & (1 << 10)) != 0;
            Logger::console("  SYSTEM_CONFIG=%X  OPERATION_INIT=%d (%s)  RSTE=%d  EEP_OFF=%d",
                             sysCfg, opInit, opInitDesc[opInit], rste, eepOff);
        } else {
            Logger::console("  SYSTEM_CONFIG: no reply");
        }
        break;
    }
    case 'u': case 'U': {
        chargerFullChargeOverride = !chargerFullChargeOverride;
        preferences.begin("settings", false);
        preferences.putBool("chgFullOvr", chargerFullChargeOverride);
        preferences.end();
        applyChargeTargetVoltage();
        if (chargerReady && desiredChargerOn && !currentFaultState) {
            if (charger.reapplyCurveNow())
                Logger::console("Full-charge override %s -- target now %fV, reapplied live", chargerFullChargeOverride ? "ON" : "OFF", chargerCurveCV);
            else
                Logger::console("Full-charge override %s -- target now %fV, but live reapply failed to confirm", chargerFullChargeOverride ? "ON" : "OFF", chargerCurveCV);
        } else {
            Logger::console("Full-charge override %s -- target now %fV, will take effect next charge cycle", chargerFullChargeOverride ? "ON" : "OFF", chargerCurveCV);
        }
        break;
    }
    case 'p': case 'P':
        if (whichDisplay == SUMMARY)//already displaying summary so toggle off
        {
            printDisplay = false;
            whichDisplay = NONE;
            Logger::console("No longer displaying pack summary");
        }
        else
        {
            whichDisplay = SUMMARY;
            printDisplay = true;
            Logger::console("Enabling pack summary display");
        }
        break;
    case 'd': case 'D':
        if (whichDisplay == DETAILS)//already displaying details so toggle off
        {
            printDisplay = false;
            whichDisplay = NONE;
            Logger::console("No longer displaying pack details");
        }
        else
        {
            whichDisplay = DETAILS;
            printDisplay = true;
            Logger::console("Enabling pack details display");
        }
        break;
    case 'j': case 'J':
        bms.printJsonData();
        /* if(whichDisplay == JSON)//already displaying json so toggle off
        {
            printDisplay = false;
            whichDisplay = NONE;
            Logger::console("No longer displaying JSON details");
        }
        else
        {
            whichDisplay = JSON;
            printDisplay = true;
            Logger::console("Enabling JSON display");
        } */
        break;
    }
}

/*
    if (SERIALCONSOLE.available())
    {
        char y = SERIALCONSOLE.read();
        switch (y)
        {
        case '1': //ascii 1
            renumberBoardIDs();  // force renumber and read out
            break;
        case '2': //ascii 2
            SERIALCONSOLE.println();
            findBoards();
            break;
        case '3': //activate cell balance for 5 seconds
            SERIALCONSOLE.println();
            SERIALCONSOLE.println("Balancing");
            cellBalance();
            break;
      case '4': //clear all faults on all boards, required after Reset or FPO (first power on)
       SERIALCONSOLE.println();
       SERIALCONSOLE.println("Clearing Faults");
       clearFaults();
      break;

      case '5': //read out the status of first board
       SERIALCONSOLE.println();
       SERIALCONSOLE.println("Reading status");
       readStatus(1);
      break;

      case '6': //Read out the limit setpoints of first board
       SERIALCONSOLE.println();
       SERIALCONSOLE.println("Reading Setpoints");
       readSetpoint(1);
       SERIALCONSOLE.println(OVolt);
       SERIALCONSOLE.println(UVolt);
       SERIALCONSOLE.println(Tset);
      break;

      case '0': //Send all boards into Sleep state
       Serial.println();
       Serial.println("Sleep Mode");
       sleepBoards();
      break;

      case '9'://Pull all boards out of Sleep state
       Serial.println();
       Serial.println("Wake Boards");
       wakeBoards();
      break;
        }
    }
 */