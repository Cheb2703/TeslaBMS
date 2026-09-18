#include "WebUI.h"
#include <WiFi.h>
#include <Preferences.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <math.h>
#include <ctype.h>
#include "Chargernpb.h"
#include "BMSModuleManager.h"
#include "SerialConsole.h"
#include "config.h"
#include "Logger.h"

WebUIManager webUI;
WebSerialTee webSerialTee;   // the single global instance SERIALCONSOLE now points at (see config.h)

extern const char WEBUI_INDEX_HTML[];   // defined at the bottom of this file (PROGMEM)

// ── Globals this module reads/writes, all declared in main.cpp ─────────────
// (Same pattern SerialConsole.cpp already uses for these -- see its top of
// file `extern` block.)
extern Preferences preferences;
extern BMSModuleManager bms;
extern EEPROMSettings settings;
extern SerialConsole console;

extern String systemName;
extern int    packsConfigured;
extern int    numFoundModules;

extern String wifiSSID;
extern String wifiPassword;
extern String apSSID;
extern String apPassword;
extern String mdnsHostname;
extern bool   wifiReconnectPending;   // main.cpp acts on this in loop(), never blocks here

extern String ftpServer;
extern String ftpUser;
extern String ftpPassword;

extern String webUsername;
extern String webPassword;

extern float balanceVoltage;
extern float balanceHyst;

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
extern bool     chargerReady;
extern bool     desiredChargerOn;   // what we WANT the charger's output to be -- see main.cpp; must be kept in sync here or enforceChargerDesiredState() will fight the WebUI's own command
extern bool     chargerCurveReapplyPending; // set here, applied in main.cpp's loop() -- see comment there for why this can't be done directly in this handler
extern float    chargerDailyTargetV;
extern float    chargerFullTargetV;
extern bool     chargerFullChargeOverride;
extern void     applyChargeTargetVoltage();

static bool s_lastFaultActive = false;   // updated each pushState(), read by the charger-output-on guard

// ── small helpers ───────────────────────────────────────────────────────────
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Arduino's String(float) prints literal "nan"/"inf" text for those values
// (e.g. from a divide-by-zero when no modules are found yet) with no
// quotes -- that's valid to print, but NOT valid JSON, and silently breaks
// JSON.parse() on every field after it client-side. Every float that goes
// into the JSON via serialized() must be routed through this first.
static String safeFloatStr(float v, int decimals) {
    if (isnan(v) || isinf(v)) v = 0.0f;
    return String(v, decimals);
}

static void sendJsonOk(AsyncWebServerRequest* request, JsonDocument& extra) {
    extra["ok"] = true;
    String out;
    serializeJson(extra, out);
    request->send(200, "application/json", out);
}

static void sendJsonOk(AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{\"ok\":true}");
}

static void sendJsonError(AsyncWebServerRequest* request, const String& error, int code = 400) {
    JsonDocument doc;
    doc["ok"] = false;
    doc["error"] = error;
    String out;
    serializeJson(doc, out);
    request->send(code, "application/json", out);
}

bool WebUIManager::checkAuth(AsyncWebServerRequest* request) {
#if !WEBUI_REQUIRE_AUTH
    (void)request;
    return true; // login disabled in config.h
#endif
    if (!request->authenticate(webUsername.c_str(), webPassword.c_str())) {
        Logger::warn("Web UI: auth failed for %s (browser sent no/wrong credentials for this request)", request->url().c_str());
        request->requestAuthentication();
        return false;
    }
    return true;
}

// ── Telemetry JSON (dashboard/charging/faults) ──────────────────────────────
String WebUIManager::buildStateJson(const DisplayData& dd) {
    JsonDocument doc;

    JsonObject sys = doc["sys"].to<JsonObject>();
    sys["name"]            = systemName;
    sys["uptimeS"]          = dd.uptimeSeconds;
    sys["packsFound"]       = dd.numModules;
    sys["packsConfigured"]  = packsConfigured;
    sys["wifiConnected"]    = (WiFi.status() == WL_CONNECTED);
    sys["staIP"]            = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
    sys["apIP"]             = WiFi.softAPIP().toString();
    sys["apSSID"]           = apSSID;
    sys["freeHeapKB"]       = ESP.getFreeHeap() / 1024;

    JsonObject pack = doc["pack"].to<JsonObject>();
    pack["voltage"]    = serialized(safeFloatStr(dd.packVoltage, 2));
    pack["cellLow"]    = serialized(safeFloatStr(dd.cellLow, 3));
    pack["cellHigh"]   = serialized(safeFloatStr(dd.cellHigh, 3));
    pack["avgTemp"]    = serialized(safeFloatStr(dd.avgTemp, 1));
    pack["soc"]        = dd.socPercent;
    pack["balancing"]  = dd.balancingCount;
    pack["goodPackets"] = dd.goodPackets;
    pack["badPackets"]  = dd.badPackets;

    JsonArray modules = doc["modules"].to<JsonArray>();
    for (int m = 0; m < 2; m++) {
        JsonObject mod = modules.add<JsonObject>();
        mod["volt"] = serialized(safeFloatStr(dd.moduleVolt[m], 3));
        mod["tPos"] = serialized(safeFloatStr(dd.tempPos[m], 1));
        mod["tNeg"] = serialized(safeFloatStr(dd.tempNeg[m], 1));
        JsonArray cells = mod["cells"].to<JsonArray>();
        for (int c = 0; c < 6; c++) {
            JsonObject cell = cells.add<JsonObject>();
            cell["v"]   = serialized(safeFloatStr(dd.cellVolt[m][c], 3));
            cell["bal"] = dd.cellBalancing[m][c];
        }
    }

    JsonObject faults = doc["faults"].to<JsonObject>();
    faults["active"] = dd.isFaulted;
    faults["count"]  = dd.activeFaultCount;
    JsonArray list = faults["list"].to<JsonArray>();
    int shown = dd.activeFaultCount < MAX_DISPLAY_FAULTS ? dd.activeFaultCount : MAX_DISPLAY_FAULTS;
    for (int i = 0; i < shown; i++) {
        JsonObject f = list.add<JsonObject>();
        f["reason"]     = dd.faultReasons[i];
        f["durationMs"] = dd.faultDurationsMs[i];
    }
    JsonObject history = faults["history"].to<JsonObject>();
    history["has"]        = dd.hasFaultHistory;
    history["reason"]     = dd.lastFaultReason;
    history["count"]      = dd.lastFaultCount;
    history["durationMs"] = dd.lastFaultDurationMs;
    history["secondsAgo"] = dd.secondsSinceFaultCleared;

    JsonObject chg = doc["charger"].to<JsonObject>();
    chg["present"]      = dd.chargerPresent;
    chg["online"]       = dd.chargerOnline;
    chg["outputOn"]     = dd.chargerOutputOn;
    chg["vout"]         = serialized(safeFloatStr(dd.chargerVout, 2));
    chg["iout"]         = serialized(safeFloatStr(dd.chargerIout, 2));
    chg["temp"]         = serialized(safeFloatStr(dd.chargerTemp, 1));
    chg["voltSetpoint"] = serialized(safeFloatStr(dd.chargerVoltSetpoint, 2));
    chg["currSetpoint"] = serialized(safeFloatStr(dd.chargerCurrSetpoint, 2));
    chg["faulted"]      = dd.chargerFaulted;
    chg["faultStr"]     = dd.chargerFaultStr;
    chg["chgStatusStr"] = ChargerNPB::chgStatusToString(dd.chargerChgStatusRaw);
    chg["lastRxAgoMs"]  = dd.chargerLastRxAgoMs;
    chg["fullChargeOverride"] = chargerFullChargeOverride;
    chg["activeTargetV"]      = serialized(safeFloatStr(chargerCurveCV, 2));

    s_lastFaultActive = dd.isFaulted;

    String out;
    serializeJson(doc, out);
    return out;
}

// ── Settings JSON (config only, no passwords) ───────────────────────────────
String WebUIManager::buildSettingsJson() {
    JsonDocument doc;
    doc["systemName"]      = systemName;
    doc["packsConfigured"] = packsConfigured;
    doc["balanceVoltage"]  = serialized(safeFloatStr(balanceVoltage, 3));
    doc["balanceHyst"]     = serialized(safeFloatStr(balanceHyst, 4));
    doc["voltLimHi"]       = serialized(safeFloatStr(settings.OverVSetpoint, 3));
    doc["voltLimLo"]       = serialized(safeFloatStr(settings.UnderVSetpoint, 3));
    doc["tempLimHi"]       = serialized(safeFloatStr(settings.OverTSetpoint, 1));
    doc["tempLimLo"]       = serialized(safeFloatStr(settings.UnderTSetpoint, 1));

    doc["wifiSSID"]  = wifiSSID;
    doc["apSSID"]    = apSSID;
    doc["mdnsHostname"] = mdnsHostname;
    doc["webUsername"] = webUsername;
    doc["ftpServer"] = ftpServer;
    doc["ftpUser"]   = ftpUser;
    // Passwords deliberately omitted -- write-only from the browser's POV.

    JsonObject chg = doc["charger"].to<JsonObject>();
    chg["voltage"]      = serialized(safeFloatStr(chargerVoltage, 2));
    chg["current"]      = serialized(safeFloatStr(chargerCurrent, 2));
    chg["curveCC"]      = serialized(safeFloatStr(chargerCurveCC, 2));
    chg["curveCV"]      = serialized(safeFloatStr(chargerCurveCV, 2));
    chg["curveFV"]      = serialized(safeFloatStr(chargerCurveFV, 2));
    chg["curveTC"]      = serialized(safeFloatStr(chargerCurveTC, 2));
    chg["restartVbat"]  = serialized(safeFloatStr(chargerRstVbat, 2));
    chg["ccTimeoutMin"] = chargerCCTimeoutMin;
    chg["cvTimeoutMin"] = chargerCVTimeoutMin;
    chg["fvTimeoutMin"] = chargerFVTimeoutMin;
    chg["dailyTargetV"] = serialized(safeFloatStr(chargerDailyTargetV, 2));
    chg["fullTargetV"]  = serialized(safeFloatStr(chargerFullTargetV, 2));

    String out;
    serializeJson(doc, out);
    return out;
}

// ── Route handlers ───────────────────────────────────────────────────────────
void WebUIManager::onIndex(AsyncWebServerRequest* request) {
    if (!checkAuth(request)) return;
    // _P variant: reads straight out of flash (PROGMEM) instead of copying
    // the whole ~15KB page into a heap String on every single request.
    AsyncWebServerResponse* response = request->beginResponse_P(200, "text/html", WEBUI_INDEX_HTML);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
}

void WebUIManager::onApiState(AsyncWebServerRequest* request) {
    if (!checkAuth(request)) return;
    request->send(200, "application/json", _lastStateJson.length() ? _lastStateJson : String("{}"));
}

void WebUIManager::onApiSettingsGet(AsyncWebServerRequest* request) {
    if (!checkAuth(request)) return;
    request->send(200, "application/json", buildSettingsJson());
}

void WebUIManager::onApiSettingsPost(AsyncWebServerRequest* request, const String& body) {
    if (!checkAuth(request)) return;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) { sendJsonError(request, "invalid JSON body"); return; }

    // Everything below is applied to RAM immediately, then written to
    // Preferences in ONE begin()/end() pass at the end -- unlike the old
    // handleUpdate(), which opened/closed the "settings" namespace once per
    // field. Fewer flash transactions, and no half-applied state if
    // something below returns early.
    preferences.begin("settings", false);

    if (doc["systemName"].is<const char*>()) {
        systemName = doc["systemName"].as<String>();
        preferences.putString("systemName", systemName);
    }
    if (doc["packsConfigured"].is<int>()) {
        int v = doc["packsConfigured"].as<int>();
        packsConfigured = v < 1 ? 1 : (v > MAX_MODULE_ADDR ? MAX_MODULE_ADDR : v);
        preferences.putInt("packsConfigured", packsConfigured);
    }
    if (doc["balanceVoltage"].is<float>()) {
        balanceVoltage = clampf(doc["balanceVoltage"].as<float>(), 2.5f, 4.3f);
        settings.balanceVoltage = balanceVoltage;
        preferences.putFloat("balanceVoltage", balanceVoltage);
    }
    if (doc["balanceHyst"].is<float>()) {
        balanceHyst = clampf(doc["balanceHyst"].as<float>(), 0.001f, 0.5f);
        settings.balanceHyst = balanceHyst;
        preferences.putFloat("balanceHyst", balanceHyst);
    }
    if (doc["voltLimHi"].is<float>()) {
        settings.OverVSetpoint = clampf(doc["voltLimHi"].as<float>(), 2.5f, 4.3f);
        preferences.putFloat("overVSetpoint", settings.OverVSetpoint);
    }
    if (doc["voltLimLo"].is<float>()) {
        settings.UnderVSetpoint = clampf(doc["voltLimLo"].as<float>(), 1.5f, 4.0f);
        preferences.putFloat("underVSetpoint", settings.UnderVSetpoint);
    }
    if (doc["tempLimHi"].is<float>()) {
        settings.OverTSetpoint = clampf(doc["tempLimHi"].as<float>(), 0.0f, 100.0f);
        preferences.putFloat("overTSetpoint", settings.OverTSetpoint);
    }
    if (doc["tempLimLo"].is<float>()) {
        settings.UnderTSetpoint = clampf(doc["tempLimLo"].as<float>(), -40.0f, 50.0f);
        preferences.putFloat("underTSetpoint", settings.UnderTSetpoint);
    }

    if (doc["ftpServer"].is<const char*>()) {
        ftpServer = doc["ftpServer"].as<String>();
        preferences.putString("ftpServer", ftpServer);
    }
    if (doc["ftpUser"].is<const char*>()) {
        ftpUser = doc["ftpUser"].as<String>();
        preferences.putString("ftpUser", ftpUser);
    }
    if (doc["ftpPassword"].is<const char*>() && doc["ftpPassword"].as<String>().length() > 0) {
        ftpPassword = doc["ftpPassword"].as<String>();
        preferences.putString("ftpPassword", ftpPassword);
    }

    if (doc["webUsername"].is<const char*>() && doc["webUsername"].as<String>().length() > 0) {
        webUsername = doc["webUsername"].as<String>();
        preferences.putString("webUsername", webUsername);
    }
    if (doc["webPassword"].is<const char*>() && doc["webPassword"].as<String>().length() > 0) {
        webPassword = doc["webPassword"].as<String>();
        preferences.putString("webPassword", webPassword);
    }

    if (doc["apSSID"].is<const char*>() && doc["apSSID"].as<String>().length() > 0) {
        apSSID = doc["apSSID"].as<String>();
        preferences.putString("apSSID", apSSID);
        // AP is applied on next boot rather than torn down live -- ripping
        // the AP down while this very request is being served over it would
        // drop the response. See README note in the settings page.
    }
    if (doc["apPassword"].is<const char*>() && doc["apPassword"].as<String>().length() >= 8) {
        apPassword = doc["apPassword"].as<String>();
        preferences.putString("apPassword", apPassword);
    }

    if (doc["mdnsHostname"].is<const char*>()) {
        String h = doc["mdnsHostname"].as<String>();
        bool validChars = h.length() > 0 && h.length() <= 32;
        for (size_t i = 0; validChars && i < h.length(); i++) {
            char c = h[i];
            if (!isalnum((unsigned char)c) && c != '-') validChars = false;
        }
        if (!validChars) {
            preferences.end();
            sendJsonError(request, "mdnsHostname must be 1-32 characters, letters/numbers/hyphens only");
            return;
        }
        mdnsHostname = h;
        preferences.putString("mdnsHostname", mdnsHostname);
        // Only takes effect on next reboot -- see the comment on mDNS.begin() in setup().
    }

    bool wifiChanged = false;
    if (doc["wifiSSID"].is<const char*>()) {
        String v = doc["wifiSSID"].as<String>();
        if (v != wifiSSID) { wifiSSID = v; preferences.putString("wifiSSID", wifiSSID); wifiChanged = true; }
    }
    if (doc["wifiPassword"].is<const char*>() && doc["wifiPassword"].as<String>().length() > 0) {
        wifiPassword = doc["wifiPassword"].as<String>();
        preferences.putString("wifiPassword", wifiPassword);
        wifiChanged = true;
    }

    preferences.end();

    if (wifiChanged) {
        wifiReconnectPending = true;   // loop() picks this up -- never block in a request handler
    }

    JsonDocument resp;
    resp["wifiReconnectScheduled"] = wifiChanged;
    sendJsonOk(request, resp);
}

void WebUIManager::onApiChargerPost(AsyncWebServerRequest* request, const String& body) {
    if (!checkAuth(request)) return;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) { sendJsonError(request, "invalid JSON body"); return; }

    if (!chargerReady) { sendJsonError(request, "charger not present / CAN link never came up at boot", 409); return; }

    preferences.begin("settings", false);

    // voltage/current (CHGV/CHGI-equivalent) and curveCV/curveCC are the SAME
    // underlying charger registers (CURVE_CV/CURVE_CC) -- VOUT_SET/IOUT_SET
    // are ignored while the charger runs its curve/battery-charging profile,
    // which is always true in this application. Both fields are kept
    // mirrored so the Charging tab and Settings tab always agree, and any of
    // curveChanged tracks whether we need to reapply the curve live below.
    bool curveChanged = false;

    if (doc["voltage"].is<float>()) {
        chargerVoltage = clampf(doc["voltage"].as<float>(), NPB24_VOLT_MIN, NPB24_VOLT_MAX);
        chargerCurveCV = chargerVoltage;
        preferences.putFloat("chgVoltage", chargerVoltage);
        preferences.putFloat("chgCurveCV", chargerCurveCV);
        charger.setCurveCV(chargerCurveCV);
        curveChanged = true;
    }
    if (doc["current"].is<float>()) {
        chargerCurrent = clampf(doc["current"].as<float>(), NPB24_CURR_MIN, NPB24_CURR_MAX);
        chargerCurveCC = chargerCurrent;
        preferences.putFloat("chgCurrent", chargerCurrent);
        preferences.putFloat("chgCurveCC", chargerCurveCC);
        charger.setCurveCC(chargerCurveCC);
        curveChanged = true;
    }
    if (doc["curveCC"].is<float>()) {
        chargerCurveCC = clampf(doc["curveCC"].as<float>(), NPB24_CURR_MIN, NPB24_CURR_MAX);
        chargerCurrent = chargerCurveCC;
        preferences.putFloat("chgCurveCC", chargerCurveCC);
        preferences.putFloat("chgCurrent", chargerCurrent);
        charger.setCurveCC(chargerCurveCC);
        curveChanged = true;
    }
    if (doc["curveCV"].is<float>()) {
        chargerCurveCV = clampf(doc["curveCV"].as<float>(), NPB24_VOLT_MIN, NPB24_VOLT_MAX);
        chargerVoltage = chargerCurveCV;
        preferences.putFloat("chgCurveCV", chargerCurveCV);
        preferences.putFloat("chgVoltage", chargerVoltage);
        charger.setCurveCV(chargerCurveCV);
        curveChanged = true;
    }
    if (doc["curveFV"].is<float>()) {
        chargerCurveFV = clampf(doc["curveFV"].as<float>(), NPB24_VOLT_MIN, NPB24_VOLT_MAX);
        preferences.putFloat("chgCurveFV", chargerCurveFV);
        charger.setCurveFV(chargerCurveFV);
        curveChanged = true;
    }
    if (doc["curveTC"].is<float>()) {
        chargerCurveTC = clampf(doc["curveTC"].as<float>(), NPB24_CURR_MIN, NPB24_CURR_MAX);
        preferences.putFloat("chgCurveTC", chargerCurveTC);
        charger.setCurveTC(chargerCurveTC);
        curveChanged = true;
    }
    // Daily-limit / full-charge-override system -- see applyChargeTargetVoltage()
    // in main.cpp. These drive chargerCurveCV/chargerCurveFV directly, same
    // idea as voltage/curveCV above but through the daily/full lens instead
    // of a raw manual value.
    if (doc["dailyTargetV"].is<float>()) {
        chargerDailyTargetV = clampf(doc["dailyTargetV"].as<float>(), NPB24_VOLT_MIN, NPB24_VOLT_MAX);
        preferences.putFloat("chgDailyV", chargerDailyTargetV);
        applyChargeTargetVoltage();
        if (!chargerFullChargeOverride) curveChanged = true; // only matters live if it's the active target right now
    }
    if (doc["fullTargetV"].is<float>()) {
        chargerFullTargetV = clampf(doc["fullTargetV"].as<float>(), NPB24_VOLT_MIN, NPB24_VOLT_MAX);
        preferences.putFloat("chgFullV", chargerFullTargetV);
        applyChargeTargetVoltage();
        if (chargerFullChargeOverride) curveChanged = true;
    }
    if (doc["fullChargeOverride"].is<bool>()) {
        chargerFullChargeOverride = doc["fullChargeOverride"].as<bool>();
        preferences.putBool("chgFullOvr", chargerFullChargeOverride);
        applyChargeTargetVoltage();
        curveChanged = true;
    }

    if (doc["restartVbat"].is<float>()) {
        chargerRstVbat = clampf(doc["restartVbat"].as<float>(), NPB24_VOLT_MIN, NPB24_VOLT_MAX);
        preferences.putFloat("chgRstVbat", chargerRstVbat);
        charger.setChgRstVbat(chargerRstVbat);
        curveChanged = true;
    }
    if (doc["ccTimeoutMin"].is<int>()) {
        chargerCCTimeoutMin = (uint16_t)constrain(doc["ccTimeoutMin"].as<int>(), 0, 6000);
        preferences.putUShort("chgCCTmout", chargerCCTimeoutMin);
        charger.setCurveCCTimeoutMinutes(chargerCCTimeoutMin);
        curveChanged = true;
    }
    if (doc["cvTimeoutMin"].is<int>()) {
        chargerCVTimeoutMin = (uint16_t)constrain(doc["cvTimeoutMin"].as<int>(), 0, 6000);
        preferences.putUShort("chgCVTmout", chargerCVTimeoutMin);
        charger.setCurveCVTimeoutMinutes(chargerCVTimeoutMin);
        curveChanged = true;
    }
    if (doc["fvTimeoutMin"].is<int>()) {
        chargerFVTimeoutMin = (uint16_t)constrain(doc["fvTimeoutMin"].as<int>(), 0, 6000);
        preferences.putUShort("chgFVTmout", chargerFVTimeoutMin);
        charger.setCurveFVTimeoutMinutes(chargerFVTimeoutMin);
        curveChanged = true;
    }

    preferences.end();

    bool outputResult = true;
    bool outputRequested = doc["outputOn"].is<bool>();
    bool justTurnedOn = false;
    if (outputRequested) {
        bool wantOn = doc["outputOn"].as<bool>();
        if (wantOn && s_lastFaultActive) {
            sendJsonError(request, "refused: an active BMS fault is holding the output OFF", 409);
            return;
        }
        desiredChargerOn = wantOn; // keep intent in sync BEFORE commanding, so the
                                   // next poll cycle's mismatch check doesn't see this
                                   // as a rogue state change and "correct" it right back
        outputResult = charger.setOutput(wantOn);
        justTurnedOn = wantOn && outputResult;
    }

    // Curve-family registers only latch in on a remote/comm on-off toggle or
    // AC power cycle -- if the charger is (now, after any outputOn field in
    // THIS same request has been applied above) supposed to be running,
    // schedule that toggle for loop() to perform. Checked after outputOn so
    // a request that changes a curve value AND turns the output off in the
    // same call doesn't schedule a toggle that would turn it right back on --
    // and skipped entirely if this same request just turned output ON, since
    // that fresh ON already picks up the curve registers written above (same
    // idea as the boot/fault-clear sequences: write registers, then turn on).
    // NEVER call ChargerNPB::reapplyCurveNow() directly here -- it blocks up
    // to ~8s, and blocking an AsyncWebServer handler is exactly what caused
    // the old UI's crash/hang-on-save bug (see wifiReconnectPending in
    // main.cpp for the same pattern). If the charger isn't currently running
    // (or a fault is active), the value is just saved for next time -- no
    // point interrupting a charger that's off.
    bool curveReapplyScheduled = false;
    if (curveChanged && desiredChargerOn && !s_lastFaultActive && !justTurnedOn) {
        curveReapplyScheduled = true;
        chargerCurveReapplyPending = true;
    }

    JsonDocument resp;
    resp["curveReapplyScheduled"] = curveReapplyScheduled;
    resp["outputCommandOk"] = outputResult;
    sendJsonOk(request, resp);
}

void WebUIManager::onApiFaultsClear(AsyncWebServerRequest* request) {
    if (!checkAuth(request)) return;
    bms.clearFaults();
    sendJsonOk(request);
}

void WebUIManager::onApiReboot(AsyncWebServerRequest* request) {
    if (!checkAuth(request)) return;
    sendJsonOk(request);
    xTaskCreate([](void*) {
        delay(500);
        esp_restart();
    }, "RebootTask", 2048, nullptr, 1, nullptr);
}

void WebUIManager::onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                              AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Logger::info("Web UI: /ws client #%d connected from %s", client->id(), client->remoteIP().toString().c_str());
        if (_lastStateJson.length()) client->text(_lastStateJson);
        else Logger::warn("Web UI: /ws client connected but no telemetry has been built yet (pushState() hasn't run)");
    } else if (type == WS_EVT_DISCONNECT) {
        Logger::info("Web UI: /ws client #%d disconnected", client->id());
    } else if (type == WS_EVT_ERROR) {
        Logger::warn("Web UI: /ws client #%d error", client->id());
    }
    // No client->server messages are used by this UI (telemetry is
    // one-directional); ignore WS_EVT_DATA etc.
}

// Console tab -- bidirectional. Browser sends a typed command line as a
// text frame; we feed it straight into SerialConsole::injectLine(), which
// runs it through the exact same parser USB input uses. Whatever that
// command logs (via Logger) shows up moments later through the normal
// pumpLog() tail -- there's no separate "response" message, it's just the
// same live output stream USB serial would show.
void WebUIManager::onLogWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                                 AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            String line;
            line.reserve(len);
            for (size_t i = 0; i < len; i++) line += (char)data[i];
            line.trim();
            if (line.length() > 0) {
                console.injectLine(line);
            }
        }
    }
}

// ── Public API ───────────────────────────────────────────────────────────────
void WebUIManager::begin(AsyncWebServer* server, AsyncWebSocket* ws) {
    _server = server;
    _ws = ws;

    _ws->onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType type,
                        void* arg, uint8_t* data, size_t len) {
        onWsEvent(s, c, type, arg, data, len);
    });
    _server->addHandler(_ws);

    _wsLog.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType type,
                          void* arg, uint8_t* data, size_t len) {
        onLogWsEvent(s, c, type, arg, data, len);
    });
    _server->addHandler(&_wsLog);
    _logCursor = webSerialTee.headCursor();   // start the Console tab live from "now", not a full backlog replay

    _server->on("/", HTTP_GET, [this](AsyncWebServerRequest* r) { onIndex(r); });
    _server->on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest* r) { r->send(204); });

    _server->on("/api/state", HTTP_GET, [this](AsyncWebServerRequest* r) { onApiState(r); });
    _server->on("/api/settings", HTTP_GET, [this](AsyncWebServerRequest* r) { onApiSettingsGet(r); });

    _server->on("/api/settings", HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            String* body = (String*)request->_tempObject;
            onApiSettingsPost(request, body ? *body : String());
            if (body) { delete body; request->_tempObject = nullptr; }
        },
        nullptr,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            if (index == 0) request->_tempObject = new String();
            ((String*)request->_tempObject)->concat((const char*)data, len);
        });

    _server->on("/api/charger", HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            String* body = (String*)request->_tempObject;
            onApiChargerPost(request, body ? *body : String());
            if (body) { delete body; request->_tempObject = nullptr; }
        },
        nullptr,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            if (index == 0) request->_tempObject = new String();
            ((String*)request->_tempObject)->concat((const char*)data, len);
        });

    _server->on("/api/faults/clear", HTTP_POST, [this](AsyncWebServerRequest* r) { onApiFaultsClear(r); });
    _server->on("/api/reboot", HTTP_POST, [this](AsyncWebServerRequest* r) { onApiReboot(r); });

    _server->on("/updatefw", HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (!checkAuth(request)) return;
            bool success = !Update.hasError();
            JsonDocument resp;
            resp["ok"] = success;
            if (!success) resp["error"] = "flash write failed, see serial log";
            String out;
            serializeJson(resp, out);
            request->send(success ? 200 : 500, "application/json", out);
            if (success) {
                xTaskCreate([](void*) {
                    delay(800);
                    esp_restart();
                }, "FwRebootTask", 2048, nullptr, 1, nullptr);
            }
        },
        [this](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
            if (!checkAuth(request)) return;
            if (!index) {
                Logger::info("Firmware upload started: %s", filename.c_str());
                Update.begin(UPDATE_SIZE_UNKNOWN);
            }
            Update.write(data, len);
            if (final) {
                Update.end(true);
                Logger::info("Firmware upload finished: %d bytes, %s", (int)(index + len),
                             Update.hasError() ? "FAILED" : "OK");
            }
        });

    Logger::info("Web UI ready.");
}

void WebUIManager::pushState(const DisplayData& dd) {
    _lastStateJson = buildStateJson(dd);

    // Quiet heartbeat (~every 10s, not every push) so the Console tab can
    // confirm telemetry is actually being built and how many /ws clients
    // are seen server-side, without spamming a line every single second.
    static uint32_t pushCount = 0;
    pushCount++;
    if (pushCount % 10 == 1) {
        Logger::info("Web UI: pushState #%d, /ws clients=%d, payload=%d bytes",
                      (int)pushCount, (int)_ws->count(), (int)_lastStateJson.length());
    }

    if (_ws->count() > 0) {
        _ws->textAll(_lastStateJson);
    }
    _ws->cleanupClients();
}

void WebUIManager::pumpLog() {
    if (_wsLog.count() == 0) {
        _logCursor = webSerialTee.headCursor();   // nobody watching -- don't let the cursor fall behind, just track "now"
        return;
    }
    String chunk = webSerialTee.drainSince(_logCursor);
    if (chunk.length() > 0) {
        _wsLog.textAll(chunk);
    }
    _wsLog.cleanupClients();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Embedded single-page app. Deliberately self-contained (no CDN, no
//  LittleFS/uploadfs step) -- this device spends nearly all its life on an
//  isolated AP with no internet, and a single PROGMEM string keeps the whole
//  UI inside the one firmware binary you already flash over USB/OTA.
// ─────────────────────────────────────────────────────────────────────────────
const char WEBUI_INDEX_HTML[] PROGMEM = R"HTMLDOC(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>Tesla ESS</title>
<style>
:root{
  --bg:#14161a; --panel:#1d2026; --panel2:#262a32; --border:#333844;
  --text:#eef0f3; --muted:#9aa2b1; --accent:#3d8bfd; --green:#2ecc71;
  --amber:#f5a623; --red:#e74c3c;
}
*{box-sizing:border-box;}
body{background:var(--bg);color:var(--text);font-family:-apple-system,Segoe UI,Roboto,Arial,sans-serif;margin:0;padding-bottom:70px;}
header{background:var(--panel);padding:14px 16px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid var(--border);position:sticky;top:0;z-index:10;}
header h1{font-size:17px;margin:0;font-weight:600;}
#connDot{width:9px;height:9px;border-radius:50%;background:var(--red);display:inline-block;margin-right:6px;}
#connDot.ok{background:var(--green);}
.wrap{max-width:720px;margin:0 auto;padding:14px;}
.card{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:14px 16px;margin-bottom:12px;}
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);margin:0 0 10px 0;font-weight:600;}
.row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid var(--border);}
.row:last-child{border-bottom:none;}
.row .k{color:var(--muted);font-size:13px;}
.row .v{font-weight:600;font-size:14px;}
.big{font-size:30px;font-weight:700;}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
.modbox{background:var(--panel2);border-radius:8px;padding:10px;}
.modbox h3{margin:0 0 6px 0;font-size:12px;color:var(--muted);font-weight:600;}
.cellgrid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;}
.cell{background:var(--panel);border-radius:6px;padding:6px;text-align:center;font-size:12px;border:1px solid var(--border);}
.cell.bal{border-color:var(--amber);color:var(--amber);}
.pill{display:inline-block;padding:2px 8px;border-radius:20px;font-size:12px;font-weight:600;}
.pill.ok{background:rgba(46,204,113,.15);color:var(--green);}
.pill.bad{background:rgba(231,76,60,.15);color:var(--red);}
.pill.warn{background:rgba(245,166,35,.15);color:var(--amber);}
.faultitem{background:rgba(231,76,60,.12);border:1px solid var(--red);border-radius:8px;padding:8px 10px;margin-bottom:8px;}
.faultitem .reason{font-weight:600;color:var(--red);}
.faultitem .dur{color:var(--muted);font-size:12px;}
button{background:var(--accent);color:#fff;border:none;border-radius:8px;padding:10px 16px;font-size:14px;font-weight:600;cursor:pointer;}
button.secondary{background:var(--panel2);color:var(--text);border:1px solid var(--border);}
button.danger{background:var(--red);}
button:active{opacity:.8;}
button:disabled{opacity:.4;cursor:default;}
label{display:block;font-size:12px;color:var(--muted);margin:10px 0 4px;}
input[type=text],input[type=password],input[type=number]{
  width:100%;background:var(--panel2);border:1px solid var(--border);color:var(--text);
  padding:9px 10px;border-radius:7px;font-size:14px;
}
input::placeholder{color:#5a6270;}
.formrow{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
.hint{font-size:11px;color:var(--muted);margin-top:4px;}
.toast{position:fixed;bottom:76px;left:50%;transform:translateX(-50%);background:var(--panel2);
  border:1px solid var(--border);padding:10px 18px;border-radius:8px;font-size:13px;opacity:0;
  transition:opacity .25s;pointer-events:none;max-width:90vw;text-align:center;z-index:50;}
.toast.show{opacity:1;}
.toast.err{border-color:var(--red);color:var(--red);}
.toast.ok{border-color:var(--green);color:var(--green);}
nav{position:fixed;bottom:0;left:0;right:0;background:var(--panel);border-top:1px solid var(--border);
  display:flex;z-index:10;}
nav button.tab{flex:1;background:transparent;border-radius:0;color:var(--muted);font-weight:500;padding:12px 4px;font-size:12px;}
nav button.tab.active{color:var(--accent);}
.section{display:none;}
.section.active{display:block;}
.progress{height:8px;background:var(--panel2);border-radius:4px;overflow:hidden;margin-top:10px;}
.progress .bar{height:100%;background:var(--accent);width:0%;transition:width .2s;}
.actionsrow{display:flex;gap:10px;margin-top:6px;flex-wrap:wrap;}
#con_out{background:#0b0d10;border:1px solid var(--border);border-radius:8px;padding:10px;
  height:56vh;min-height:280px;overflow-y:auto;font-family:Menlo,Consolas,monospace;font-size:12.5px;
  white-space:pre-wrap;word-break:break-word;line-height:1.45;}
#con_out .ln{color:#c9d1d9;}
#con_out .cmd{color:var(--accent);}
</style>
</head>
<body>

<header>
  <h1>&#x1F50B; <span id="hdrName">Tesla ESS</span></h1>
  <div><span id="connDot"></span><span id="connText" style="font-size:12px;color:var(--muted)">connecting...</span></div>
</header>

<div class="wrap">

  <!-- ───────── Dashboard ───────── -->
  <section id="tab-dash" class="section active">
    <div class="card">
      <h2>Pack</h2>
      <div class="row"><span class="k">Pack Voltage</span><span class="v big" id="d_packV">--</span></div>
      <div class="row"><span class="k">Estimated SoC</span><span class="v" id="d_soc">--</span></div>
      <div class="row"><span class="k">Cell Low / High</span><span class="v" id="d_cellrange">--</span></div>
      <div class="row"><span class="k">Avg Temp</span><span class="v" id="d_temp">--</span></div>
      <div class="row"><span class="k">Cells Balancing</span><span class="v" id="d_bal">--</span></div>
      <div class="row"><span class="k">Modules Found</span><span class="v" id="d_modules">--</span></div>
      <div class="row"><span class="k">Comms (good/bad)</span><span class="v" id="d_comms">--</span></div>
      <div class="row"><span class="k">Uptime</span><span class="v" id="d_uptime">--</span></div>
    </div>

    <div class="card">
      <h2>Modules</h2>
      <div class="grid2" id="d_moduleGrid"></div>
    </div>

    <div class="card">
      <h2>Network</h2>
      <div class="row"><span class="k">Access Point</span><span class="v" id="d_apinfo">--</span></div>
      <div class="row"><span class="k">Home Wi-Fi</span><span class="v" id="d_wifiinfo">--</span></div>
      <div class="row"><span class="k">Free Heap</span><span class="v" id="d_heap">--</span></div>
    </div>
  </section>

  <!-- ───────── Charging ───────── -->
  <section id="tab-charge" class="section">
    <div class="card">
      <h2>Charger Status</h2>
      <div class="row"><span class="k">Present</span><span class="v" id="c_present">--</span></div>
      <div class="row"><span class="k">Output</span><span class="v" id="c_output">--</span></div>
      <div class="row"><span class="k">Output Voltage</span><span class="v" id="c_vout">--</span></div>
      <div class="row"><span class="k">Output Current</span><span class="v" id="c_iout">--</span></div>
      <div class="row"><span class="k">Charger Temp</span><span class="v" id="c_temp">--</span></div>
      <div class="row"><span class="k">Stage</span><span class="v" id="c_stage">--</span></div>
      <div class="row"><span class="k">Fault</span><span class="v" id="c_fault">--</span></div>
      <div class="row"><span class="k">Last CAN reply</span><span class="v" id="c_rx">--</span></div>
      <div class="actionsrow">
        <button id="c_onBtn" class="secondary">Output ON</button>
        <button id="c_offBtn" class="secondary">Output OFF</button>
      </div>
    </div>

    <div class="card">
      <h2>Charge Target</h2>
      <div class="row"><span class="k">Mode</span><span class="v" id="c_chargeMode">--</span></div>
      <div class="row"><span class="k">Active Target</span><span class="v" id="c_activeTarget">--</span></div>
      <div class="hint">Daily mode charges to a partial-SOC target (easier on the cells for everyday use). Full-charge override pushes to the full target once, then automatically drops back to daily mode when it reports fully charged.</div>
      <div class="actionsrow">
        <button id="c_fullChargeBtn" class="secondary">Full-Charge Override: OFF</button>
      </div>
    </div>

    <div class="card">
      <h2>Setpoints</h2>
      <label>Live Output Voltage (V)</label>
      <input type="number" step="0.01" id="c_voltage">
      <label>Live Output Current Limit (A)</label>
      <input type="number" step="0.01" id="c_current">
      <div class="formrow">
        <div><label>Daily Charge Target (V)</label><input type="number" step="0.01" id="c_dailyTargetV"></div>
        <div><label>Full-Charge Target (V)</label><input type="number" step="0.01" id="c_fullTargetV"></div>
      </div>
      <div class="formrow">
        <div><label>Curve CC (A)</label><input type="number" step="0.01" id="c_curveCC"></div>
        <div><label>Curve CV (V)</label><input type="number" step="0.01" id="c_curveCV"></div>
        <div><label>Curve FV (V)</label><input type="number" step="0.01" id="c_curveFV"></div>
        <div><label>Curve TC (A)</label><input type="number" step="0.01" id="c_curveTC"></div>
      </div>
      <div class="hint">Curve CV/FV are driven by the Daily/Full-Charge targets above -- edit them there. They're still shown/editable here for advanced ad-hoc testing, but get overwritten the next time the daily/full system re-asserts itself (boot, override toggle, or full-charge completion).</div>
      <label>Auto-restart-charge Voltage (V)</label>
      <input type="number" step="0.01" id="c_restartVbat">
      <div class="formrow">
        <div><label>CC Timeout (min, 0=off)</label><input type="number" id="c_ccTimeout"></div>
        <div><label>CV Timeout (min, 0=off)</label><input type="number" id="c_cvTimeout"></div>
        <div><label>FV Timeout (min, 0=off)</label><input type="number" id="c_fvTimeout"></div>
      </div>
      <div class="hint">Curve/timeout changes take effect on the charger's next remote on/off toggle or AC power cycle -- not instantly. Live voltage/current apply immediately.</div>
      <div class="actionsrow"><button id="c_saveBtn">Save Charger Settings</button></div>
    </div>
  </section>

  <!-- ───────── Faults ───────── -->
  <section id="tab-faults" class="section">
    <div class="card">
      <h2>Active Faults</h2>
      <div id="f_active"><div class="row"><span class="k">None -- pack is healthy</span></div></div>
      <div class="actionsrow"><button id="f_clearBtn" class="danger">Clear Faults</button></div>
    </div>
    <div class="card">
      <h2>Last Cleared Fault</h2>
      <div id="f_history"><div class="row"><span class="k">No fault history yet this boot</span></div></div>
    </div>
  </section>

  <!-- ───────── Settings ───────── -->
  <section id="tab-settings" class="section">
    <div class="card">
      <h2>System</h2>
      <label>System Name</label>
      <input type="text" id="s_systemName">
      <label>Packs Configured</label>
      <input type="number" id="s_packsConfigured">
      <label>Balance Voltage (V)</label>
      <input type="number" step="0.001" id="s_balanceVoltage">
      <label>Balance Hysteresis (V)</label>
      <input type="number" step="0.001" id="s_balanceHyst">
    </div>

    <div class="card">
      <h2>Fault Limits</h2>
      <div class="formrow">
        <div><label>Cell Volt High</label><input type="number" step="0.01" id="s_voltLimHi"></div>
        <div><label>Cell Volt Low</label><input type="number" step="0.01" id="s_voltLimLo"></div>
        <div><label>Temp High (&deg;C)</label><input type="number" step="0.1" id="s_tempLimHi"></div>
        <div><label>Temp Low (&deg;C)</label><input type="number" step="0.1" id="s_tempLimLo"></div>
      </div>
    </div>

    <div class="card">
      <h2>Access Point (this device's own Wi-Fi)</h2>
      <label>AP Name (SSID)</label>
      <input type="text" id="s_apSSID">
      <label>AP Password</label>
      <input type="password" id="s_apPassword" placeholder="leave blank to keep current">
      <div class="hint">Applies on next reboot. Minimum 8 characters if you're changing it.</div>
      <label>Device Hostname</label>
      <input type="text" id="s_mdnsHostname" placeholder="e.g. Lift">
      <div class="hint">Two ways to reach this device by name instead of an IP: http://&lt;name&gt;.iot works on any device while connected to this AP (plain DNS, no dependencies). http://&lt;name&gt;.local works via mDNS/Bonjour -- built into iOS/macOS, needs Bonjour on Windows -- and unlike .iot it can also work over the home Wi-Fi, not just the AP. Both apply on next reboot.</div>
    </div>

    <div class="card">
      <h2>Home Wi-Fi (optional fallback)</h2>
      <label>SSID</label>
      <input type="text" id="s_wifiSSID">
      <label>Password</label>
      <input type="password" id="s_wifiPassword" placeholder="leave blank to keep current">
      <div class="hint">The AP stays up either way -- this only lets the device also join a home network when one's in range.</div>
    </div>

    <div class="card">
      <h2>Web UI Login</h2>
      <label>Username</label>
      <input type="text" id="s_webUsername">
      <label>Password</label>
      <input type="password" id="s_webPassword" placeholder="leave blank to keep current">
    </div>

    <div class="card">
      <h2>FTP (stats export, optional)</h2>
      <label>Server</label>
      <input type="text" id="s_ftpServer">
      <label>Username</label>
      <input type="text" id="s_ftpUser">
      <label>Password</label>
      <input type="password" id="s_ftpPassword" placeholder="leave blank to keep current">
    </div>

    <div class="actionsrow">
      <button id="s_saveBtn">Save Settings</button>
      <button id="s_rebootBtn" class="secondary">Reboot</button>
    </div>
  </section>

  <!-- ───────── Firmware ───────── -->
  <section id="tab-fw" class="section">
    <div class="card">
      <h2>Firmware Update</h2>
      <div class="hint">Upload a .bin built for this board. The device reboots automatically when the flash write succeeds.</div>
      <input type="file" id="fw_file" style="margin-top:10px;">
      <div class="progress"><div class="bar" id="fw_bar"></div></div>
      <div class="actionsrow"><button id="fw_upload">Upload</button></div>
    </div>
  </section>

  <!-- ───────── Console ───────── -->
  <section id="tab-console" class="section">
    <div class="card" style="padding:10px;">
      <h2 style="margin-left:6px;">Serial Console</h2>
      <div class="hint" style="margin:0 0 8px 6px;">Live tail of the same output USB serial shows, plus every command this menu supports (type <b>h</b> for the list).</div>
      <div id="con_out"></div>
      <div class="actionsrow" style="margin-top:8px;">
        <input type="text" id="con_in" placeholder="type a command, press Enter" style="flex:1;font-family:monospace;">
        <button id="con_send">Send</button>
        <button id="con_clear" class="secondary">Clear</button>
      </div>
    </div>
  </section>

</div>

<nav>
  <button class="tab active" data-tab="tab-dash">Dashboard</button>
  <button class="tab" data-tab="tab-charge">Charging</button>
  <button class="tab" data-tab="tab-faults">Faults</button>
  <button class="tab" data-tab="tab-settings">Settings</button>
  <button class="tab" data-tab="tab-fw">Firmware</button>
  <button class="tab" data-tab="tab-console">Console</button>
</nav>

<div class="toast" id="toast"></div>

<script>
// ── tab switching ────────────────────────────────────────────────────────
document.querySelectorAll('nav .tab').forEach(btn=>{
  btn.addEventListener('click', ()=>{
    document.querySelectorAll('nav .tab').forEach(b=>b.classList.remove('active'));
    document.querySelectorAll('.section').forEach(s=>s.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById(btn.dataset.tab).classList.add('active');
    // Console's WebSocket connects lazily, on first visit to that tab --
    // not on page load. Keeps the number of simultaneous connections down
    // during normal dashboard viewing (fewer concurrent connections means
    // less chance of tripping a known race-condition crash in the
    // underlying web server library -- see connectConsoleWs()).
    if (btn.dataset.tab === 'tab-console' && !conWs) connectConsoleWs();
  });
});

function toast(msg, kind){
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = 'toast show' + (kind ? ' '+kind : '');
  clearTimeout(toast._h);
  toast._h = setTimeout(()=>{ t.classList.remove('show'); }, 3200);
}

function fmtMs(ms){
  if (ms === undefined || ms === null) return '--';
  let s = Math.floor(ms/1000);
  const h = Math.floor(s/3600); s%=3600;
  const m = Math.floor(s/60); s%=60;
  return (h?h+'h ':'') + (m?m+'m ':'') + s+'s';
}
function fmtUptime(sec){
  const d = Math.floor(sec/86400); sec%=86400;
  const h = Math.floor(sec/3600); sec%=3600;
  const m = Math.floor(sec/60);
  return (d?d+'d ':'')+h+'h '+m+'m';
}

// ── live telemetry (WebSocket, falls back to polling) ───────────────────
let ws, wsRetryMs = 1000, settingsLoaded = false, chargerFieldsFocused = false;

function applyState(d){
  // Set first, unconditionally -- a JS error further down (bad/renamed
  // field, etc.) should not leave this stuck on "connecting..." forever,
  // and the actual error still gets logged to the browser console below
  // instead of silently disappearing.
  document.getElementById('connDot').classList.add('ok');
  document.getElementById('connText').textContent = 'live';

  try {
  document.getElementById('hdrName').textContent = d.sys.name || 'Tesla ESS';

  document.getElementById('d_packV').textContent = d.pack.voltage + ' V';
  document.getElementById('d_soc').textContent = d.pack.soc + ' %';
  document.getElementById('d_cellrange').textContent = d.pack.cellLow + ' / ' + d.pack.cellHigh + ' V';
  document.getElementById('d_temp').textContent = d.pack.avgTemp + ' \u00b0C';
  document.getElementById('d_bal').textContent = d.pack.balancing;
  document.getElementById('d_modules').textContent = d.sys.packsFound + ' / ' + d.sys.packsConfigured;
  document.getElementById('d_comms').textContent = d.pack.goodPackets + ' / ' + d.pack.badPackets;
  document.getElementById('d_uptime').textContent = fmtUptime(d.sys.uptimeS);
  document.getElementById('d_apinfo').textContent = d.sys.apSSID + ' (' + d.sys.apIP + ')';
  document.getElementById('d_wifiinfo').textContent = d.sys.wifiConnected ? d.sys.staIP : 'not connected';
  document.getElementById('d_heap').textContent = d.sys.freeHeapKB + ' KB';

  const mg = document.getElementById('d_moduleGrid');
  mg.innerHTML = '';
  d.modules.forEach((m, idx)=>{
    const box = document.createElement('div');
    box.className = 'modbox';
    let cellsHtml = '<div class="cellgrid">';
    m.cells.forEach((c,i)=>{
      cellsHtml += '<div class="cell'+(c.bal?' bal':'')+'">C'+(i+1)+'<br>'+c.v+'V</div>';
    });
    cellsHtml += '</div>';
    box.innerHTML = '<h3>Module '+(idx+1)+' &mdash; '+m.volt+'V &nbsp; '+m.tPos+'/'+m.tNeg+'\u00b0C</h3>'+cellsHtml;
    mg.appendChild(box);
  });

  // Charging tab
  document.getElementById('c_present').innerHTML = d.charger.present
    ? '<span class="pill ok">present</span>' : '<span class="pill bad">not detected</span>';
  document.getElementById('c_output').innerHTML = d.charger.outputOn
    ? '<span class="pill ok">ON</span>' : '<span class="pill warn">OFF</span>';
  document.getElementById('c_vout').textContent = d.charger.vout + ' V';
  document.getElementById('c_iout').textContent = d.charger.iout + ' A';
  document.getElementById('c_temp').textContent = d.charger.temp + ' \u00b0C';
  document.getElementById('c_stage').textContent = d.charger.chgStatusStr;
  document.getElementById('c_fault').innerHTML = d.charger.faulted
    ? '<span class="pill bad">'+d.charger.faultStr+'</span>' : '<span class="pill ok">OK</span>';
  document.getElementById('c_rx').textContent = d.charger.online ? fmtMs(d.charger.lastRxAgoMs)+' ago' : 'no link';
  document.getElementById('c_chargeMode').innerHTML = d.charger.fullChargeOverride
    ? '<span class="pill warn">FULL CHARGE OVERRIDE</span>' : '<span class="pill ok">Daily limit</span>';
  document.getElementById('c_activeTarget').textContent = d.charger.activeTargetV + ' V';
  const fcBtn = document.getElementById('c_fullChargeBtn');
  fcBtn.textContent = 'Full-Charge Override: ' + (d.charger.fullChargeOverride ? 'ON' : 'OFF');
  fcBtn.classList.toggle('danger', d.charger.fullChargeOverride);
  if (!chargerFieldsFocused) {
    document.getElementById('c_voltage').value = d.charger.voltSetpoint;
    document.getElementById('c_current').value = d.charger.currSetpoint;
  }

  // Faults tab
  const fa = document.getElementById('f_active');
  if (d.faults.active && d.faults.list.length) {
    fa.innerHTML = d.faults.list.map(f=>
      '<div class="faultitem"><div class="reason">'+f.reason+'</div><div class="dur">active '+fmtMs(f.durationMs)+'</div></div>'
    ).join('') + (d.faults.count > d.faults.list.length ? '<div class="hint">+'+(d.faults.count-d.faults.list.length)+' more (see serial console)</div>' : '');
  } else {
    fa.innerHTML = '<div class="row"><span class="k">None -- pack is healthy</span></div>';
  }
  const fh = document.getElementById('f_history');
  fh.innerHTML = d.faults.history.has
    ? '<div class="row"><span class="k">'+d.faults.history.reason+'</span><span class="v">'+fmtMs(d.faults.history.durationMs)+'</span></div>'+
      '<div class="row"><span class="k">Cleared</span><span class="v">'+d.faults.history.secondsAgo+'s ago</span></div>'
    : '<div class="row"><span class="k">No fault history yet this boot</span></div>';
  } catch (e) {
    console.error('applyState: failed partway through rendering telemetry', e, d);
  }
}

function connectWs(){
  ws = new WebSocket((location.protocol==='https:'?'wss://':'ws://') + location.host + '/ws');
  ws.onopen = ()=>{
    wsRetryMs = 1000;
    document.getElementById('connText').textContent = 'connected, waiting for data...';
  };
  ws.onmessage = (evt)=>{
    try{ applyState(JSON.parse(evt.data)); }
    catch(e){ console.error('WS message was not valid JSON', e, evt.data); }
  };
  ws.onclose = (evt)=>{
    console.error('WS /ws closed', evt.code, evt.reason);
    document.getElementById('connDot').classList.remove('ok');
    document.getElementById('connText').textContent = 'reconnecting...';
    setTimeout(connectWs, wsRetryMs);
    wsRetryMs = Math.min(wsRetryMs*1.5, 10000);
  };
  ws.onerror = (e)=>{ console.error('WS /ws error', e); ws.close(); };
}

fetch('/api/state').then(r=>r.json()).then(applyState).catch(e=> console.error('Initial /api/state fetch failed', e));
connectWs();

// ── settings load/save ───────────────────────────────────────────────────
function fetchJson(url, opts){
  return fetch(url, opts).then(async r=>{
    if (!r.ok) throw new Error('HTTP ' + r.status + ' ' + r.statusText);
    return r.json();
  });
}

function loadSettings(){
  fetchJson('/api/settings').then(s=>{
    document.getElementById('s_systemName').value = s.systemName;
    document.getElementById('s_packsConfigured').value = s.packsConfigured;
    document.getElementById('s_balanceVoltage').value = s.balanceVoltage;
    document.getElementById('s_balanceHyst').value = s.balanceHyst;
    document.getElementById('s_voltLimHi').value = s.voltLimHi;
    document.getElementById('s_voltLimLo').value = s.voltLimLo;
    document.getElementById('s_tempLimHi').value = s.tempLimHi;
    document.getElementById('s_tempLimLo').value = s.tempLimLo;
    document.getElementById('s_apSSID').value = s.apSSID;
    document.getElementById('s_mdnsHostname').value = s.mdnsHostname;
    document.getElementById('s_wifiSSID').value = s.wifiSSID;
    document.getElementById('s_webUsername').value = s.webUsername;
    document.getElementById('s_ftpServer').value = s.ftpServer;
    document.getElementById('s_ftpUser').value = s.ftpUser;

    document.getElementById('c_curveCC').value = s.charger.curveCC;
    document.getElementById('c_curveCV').value = s.charger.curveCV;
    document.getElementById('c_curveFV').value = s.charger.curveFV;
    document.getElementById('c_curveTC').value = s.charger.curveTC;
    document.getElementById('c_dailyTargetV').value = s.charger.dailyTargetV;
    document.getElementById('c_fullTargetV').value = s.charger.fullTargetV;
    document.getElementById('c_restartVbat').value = s.charger.restartVbat;
    document.getElementById('c_ccTimeout').value = s.charger.ccTimeoutMin;
    document.getElementById('c_cvTimeout').value = s.charger.cvTimeoutMin;
    document.getElementById('c_fvTimeout').value = s.charger.fvTimeoutMin;
    settingsLoaded = true;
  }).catch(e=>{
    console.error('loadSettings failed', e);
    toast('Could not load settings: ' + e.message, 'err');
  });
}
loadSettings();

['c_voltage','c_current'].forEach(id=>{
  const el = document.getElementById(id);
  el.addEventListener('focus', ()=> chargerFieldsFocused = true);
  el.addEventListener('blur',  ()=> chargerFieldsFocused = false);
});

function postJson(url, body){
  return fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)})
    .then(async r=>{
      const j = await r.json().catch(()=>({ok:false,error:'bad response'}));
      if (!r.ok || !j.ok) throw new Error(j.error || ('HTTP '+r.status));
      return j;
    });
}

document.getElementById('s_saveBtn').addEventListener('click', ()=>{
  const body = {
    systemName: document.getElementById('s_systemName').value,
    packsConfigured: parseInt(document.getElementById('s_packsConfigured').value),
    balanceVoltage: parseFloat(document.getElementById('s_balanceVoltage').value),
    balanceHyst: parseFloat(document.getElementById('s_balanceHyst').value),
    voltLimHi: parseFloat(document.getElementById('s_voltLimHi').value),
    voltLimLo: parseFloat(document.getElementById('s_voltLimLo').value),
    tempLimHi: parseFloat(document.getElementById('s_tempLimHi').value),
    tempLimLo: parseFloat(document.getElementById('s_tempLimLo').value),
    apSSID: document.getElementById('s_apSSID').value,
    mdnsHostname: document.getElementById('s_mdnsHostname').value,
    wifiSSID: document.getElementById('s_wifiSSID').value,
    webUsername: document.getElementById('s_webUsername').value,
    ftpServer: document.getElementById('s_ftpServer').value,
    ftpUser: document.getElementById('s_ftpUser').value,
  };
  const apPass = document.getElementById('s_apPassword').value;
  const wifiPass = document.getElementById('s_wifiPassword').value;
  const webPass = document.getElementById('s_webPassword').value;
  const ftpPass = document.getElementById('s_ftpPassword').value;
  if (apPass) body.apPassword = apPass;
  if (wifiPass) body.wifiPassword = wifiPass;
  if (webPass) body.webPassword = webPass;
  if (ftpPass) body.ftpPassword = ftpPass;

  postJson('/api/settings', body).then(j=>{
    toast(j.wifiReconnectScheduled ? 'Saved -- reconnecting to Wi-Fi...' : 'Settings saved', 'ok');
    document.getElementById('s_apPassword').value = '';
    document.getElementById('s_wifiPassword').value = '';
    document.getElementById('s_webPassword').value = '';
    document.getElementById('s_ftpPassword').value = '';
  }).catch(e=> toast('Save failed: '+e.message, 'err'));
});

document.getElementById('s_rebootBtn').addEventListener('click', ()=>{
  if (!confirm('Reboot the device now?')) return;
  postJson('/api/reboot', {}).then(()=> toast('Rebooting...', 'ok')).catch(e=> toast(e.message,'err'));
});

// ── charger tab actions ──────────────────────────────────────────────────
document.getElementById('c_saveBtn').addEventListener('click', ()=>{
  const body = {
    voltage: parseFloat(document.getElementById('c_voltage').value),
    current: parseFloat(document.getElementById('c_current').value),
    dailyTargetV: parseFloat(document.getElementById('c_dailyTargetV').value),
    fullTargetV: parseFloat(document.getElementById('c_fullTargetV').value),
    curveCC: parseFloat(document.getElementById('c_curveCC').value),
    curveCV: parseFloat(document.getElementById('c_curveCV').value),
    curveFV: parseFloat(document.getElementById('c_curveFV').value),
    curveTC: parseFloat(document.getElementById('c_curveTC').value),
    restartVbat: parseFloat(document.getElementById('c_restartVbat').value),
    ccTimeoutMin: parseInt(document.getElementById('c_ccTimeout').value),
    cvTimeoutMin: parseInt(document.getElementById('c_cvTimeout').value),
    fvTimeoutMin: parseInt(document.getElementById('c_fvTimeout').value),
  };
  postJson('/api/charger', body).then(()=> toast('Charger settings saved','ok')).catch(e=> toast('Failed: '+e.message,'err'));
});
document.getElementById('c_onBtn').addEventListener('click', ()=>{
  postJson('/api/charger', {outputOn:true}).then(()=> toast('Output ON commanded','ok')).catch(e=> toast(e.message,'err'));
});
document.getElementById('c_offBtn').addEventListener('click', ()=>{
  postJson('/api/charger', {outputOn:false}).then(()=> toast('Output OFF commanded','ok')).catch(e=> toast(e.message,'err'));
});
document.getElementById('c_fullChargeBtn').addEventListener('click', ()=>{
  const turningOn = !document.getElementById('c_fullChargeBtn').textContent.includes('ON');
  postJson('/api/charger', {fullChargeOverride: turningOn})
    .then(()=> toast(turningOn ? 'Full-charge override ON' : 'Full-charge override OFF -- back to daily limit', 'ok'))
    .catch(e=> toast(e.message,'err'));
});

// ── faults tab ────────────────────────────────────────────────────────────
document.getElementById('f_clearBtn').addEventListener('click', ()=>{
  postJson('/api/faults/clear', {}).then(()=> toast('Fault clear sent to BMBs','ok')).catch(e=> toast(e.message,'err'));
});

// ── firmware tab ──────────────────────────────────────────────────────────
document.getElementById('fw_upload').addEventListener('click', ()=>{
  const f = document.getElementById('fw_file').files[0];
  if (!f) { toast('Choose a .bin file first', 'err'); return; }
  const xhr = new XMLHttpRequest();
  const fd = new FormData();
  fd.append('firmware', f);
  xhr.upload.addEventListener('progress', (e)=>{
    if (e.lengthComputable) document.getElementById('fw_bar').style.width = Math.round(e.loaded/e.total*100)+'%';
  });
  xhr.onload = ()=>{
    try{
      const j = JSON.parse(xhr.responseText);
      if (j.ok) toast('Upload OK -- rebooting...', 'ok'); else toast('Upload failed: '+j.error, 'err');
    }catch(e){ toast(xhr.status===200 ? 'Upload OK -- rebooting...' : 'Upload failed', xhr.status===200?'ok':'err'); }
  };
  xhr.onerror = ()=> toast('Upload failed (connection lost)', 'err');
  xhr.open('POST', '/updatefw');
  xhr.send(fd);
});

// ── console tab ───────────────────────────────────────────────────────────
const MAX_CON_LINES = 600;
let conWs, conRetryMs = 1000, conLineCount = 0, conHistory = [], conHistPos = -1;
const conOut = document.getElementById('con_out');

function conAppendRaw(text, cls){
  // Incoming chunks may contain partial lines / multiple lines / \r\n --
  // normalize and append as individual line elements so trimming old lines
  // (MAX_CON_LINES) is simple, and so a typed command can be styled
  // differently from device output.
  const parts = text.split(/\r\n|\n|\r/);
  parts.forEach((p, idx)=>{
    if (p.length === 0 && idx < parts.length-1) return; // drop empty lines from the split, keep a trailing partial
    const div = document.createElement('div');
    div.className = 'ln' + (cls ? ' '+cls : '');
    div.textContent = p;
    conOut.appendChild(div);
    conLineCount++;
  });
  while (conLineCount > MAX_CON_LINES && conOut.firstChild) {
    conOut.removeChild(conOut.firstChild);
    conLineCount--;
  }
  conOut.scrollTop = conOut.scrollHeight;
}

function connectConsoleWs(){
  conWs = new WebSocket((location.protocol==='https:'?'wss://':'ws://') + location.host + '/wslog');
  conWs.onopen = ()=>{ conRetryMs = 1000; conAppendRaw('[connected]'); };
  conWs.onmessage = (evt)=> conAppendRaw(evt.data);
  conWs.onclose = ()=>{
    conAppendRaw('[disconnected -- retrying]');
    setTimeout(connectConsoleWs, conRetryMs);
    conRetryMs = Math.min(conRetryMs*1.5, 10000);
  };
  conWs.onerror = ()=>{ conWs.close(); };
}
// Connected lazily from the tab-click handler above, not here on page load.

function conSend(){
  const el = document.getElementById('con_in');
  const line = el.value;
  if (!line.length) return;
  if (!conWs || conWs.readyState !== WebSocket.OPEN) { toast('Console not connected', 'err'); return; }
  conAppendRaw('> ' + line, 'cmd');
  conWs.send(line);
  conHistory.push(line);
  if (conHistory.length > 50) conHistory.shift();
  conHistPos = conHistory.length;
  el.value = '';
}
document.getElementById('con_send').addEventListener('click', conSend);
document.getElementById('con_in').addEventListener('keydown', (e)=>{
  if (e.key === 'Enter') { conSend(); return; }
  if (e.key === 'ArrowUp') {
    if (conHistPos > 0) { conHistPos--; e.target.value = conHistory[conHistPos]; }
    e.preventDefault();
  } else if (e.key === 'ArrowDown') {
    if (conHistPos < conHistory.length - 1) { conHistPos++; e.target.value = conHistory[conHistPos]; }
    else { conHistPos = conHistory.length; e.target.value = ''; }
    e.preventDefault();
  }
});
document.getElementById('con_clear').addEventListener('click', ()=>{
  conOut.innerHTML = ''; conLineCount = 0;
});
</script>
</body>
</html>
)HTMLDOC";