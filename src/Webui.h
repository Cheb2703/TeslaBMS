#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Displaymanager.h"   // for DisplayData

// ─────────────────────────────────────────────────────────────────────────────
//  WebSerialTee -- drop-in replacement for the SERIALCONSOLE macro target
//  (see config.h). Every SERIALCONSOLE.print()/println()/write() call --
//  that's Logger's debug/info/warn/error/console() output, plus
//  SerialConsole's own menu text and input echo -- gets mirrored into a
//  small ring buffer here IN ADDITION to going out the real USB Serial port
//  exactly as before. begin()/available()/read()/peek() all just forward
//  straight through, so the USB serial console keeps working unchanged.
//
//  WebUIManager::pumpLog() drains this ring buffer once per loop() and
//  forwards new bytes to the "/wslog" WebSocket, giving the web Console tab
//  a near-real-time tail of the same output you'd see over USB.
//
//  SCOPE NOTE: this only captures what actually flows through SERIALCONSOLE.
//  A handful of scattered `Serial.println(...)` debug lines elsewhere in the
//  codebase (mostly boot-time Wi-Fi status, before the web UI even exists
//  yet) call the real Serial object directly and won't show up here.
//
//  Lives here (rather than its own file) because config.h needs the full
//  class definition wherever SERIALCONSOLE is used -- config.h already
//  includes this header for that reason, which is also why WebUI.h ends up
//  a dependency of config.h now. Worth knowing if you're ever chasing build
//  times: config.h now transitively pulls in ESPAsyncWebServer + LovyanGFX
//  (via Displaymanager.h) for every file in the project, not just this one.
// ─────────────────────────────────────────────────────────────────────────────
#define WEBLOG_BUF_SIZE 4096

class WebSerialTee : public Stream {
public:
    void begin(unsigned long baud) { Serial.begin(baud); }

    // -- input side: pure passthrough, USB serial input is unaffected --
    int available() override { return Serial.available(); }
    int read() override      { return Serial.read(); }
    int peek() override      { return Serial.peek(); }

    // -- output side: forward to the real port AND capture into the ring --
    size_t write(uint8_t c) override {
        _buf[_head % WEBLOG_BUF_SIZE] = c;
        _head++;
        return Serial.write(c);
    }
    size_t write(const uint8_t* buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            _buf[_head % WEBLOG_BUF_SIZE] = buffer[i];
            _head++;
        }
        return Serial.write(buffer, size);
    }

    // Returns everything written since `cursor`, and advances it to "now".
    // If the caller fell behind by more than the buffer size, silently
    // drops the oldest overrun bytes (a live tail, not a full history) and
    // snaps the cursor forward rather than replaying garbage.
    String drainSince(uint32_t& cursor) {
        uint32_t avail = _head - cursor;   // relies on uint32_t wraparound; fine for uptimes well beyond this device's realistic runtime
        if (avail == 0) return String();
        if (avail > WEBLOG_BUF_SIZE) {
            cursor = _head - WEBLOG_BUF_SIZE;
            avail = WEBLOG_BUF_SIZE;
        }
        String out;
        out.reserve(avail);
        for (uint32_t i = 0; i < avail; i++) {
            out += (char)_buf[(cursor + i) % WEBLOG_BUF_SIZE];
        }
        cursor = _head;
        return out;
    }

    uint32_t headCursor() const { return _head; }

private:
    uint8_t  _buf[WEBLOG_BUF_SIZE];
    uint32_t _head = 0;
};

extern WebSerialTee webSerialTee;

// ─────────────────────────────────────────────────────────────────────────────
//  WebUIManager -- owns every HTTP/WebSocket route for the on-device web app.
//
//  Design goals (this replaces the old string-concatenated handleRoot() /
//  handleUpdate() in main.cpp):
//    - One single-page app (embedded HTML/CSS/JS, PROGMEM, no CDN/internet
//      dependency -- this thing spends its life on an isolated AP) served
//      at "/". Tabs: Dashboard, Charging, Faults, Settings, Firmware.
//    - Live telemetry (cells, temps, charging, faults) goes out over the
//      WebSocket ("/ws") once a second, driven by pushState() being called
//      from loop() using the SAME DisplayData struct the LCD already builds
//      -- one source of truth, no duplicate polling/formatting logic.
//    - Settings/charger writes are JSON POSTs (fetch(), no full-page
//      reload), applied and saved to Preferences in ONE atomic
//      begin()/end() pass per request, and always answered with a JSON
//      {"ok":true/false,"error":"..."} body so the page can show real
//      success/failure instead of guessing.
//    - Nothing here blocks inside a request handler waiting on external
//      state (the old code's `while (WiFi.status() != WL_CONNECTED)` loop
//      inside handleUpdate() -- run from the AsyncWebServer/lwIP task --
//      is the most likely source of the "crashes sometimes" symptom). A
//      WiFi-credential change instead sets a flag that loop() acts on.
//    - Passwords are write-only: GET /api/settings never echoes back
//      wifiPassword/webPassword/apPassword. The UI shows them
//      as blank with a "leave blank to keep" placeholder; blank on submit
//      means "don't change".
// ─────────────────────────────────────────────────────────────────────────────
class WebUIManager {
public:
    // Wires up every route on the given (already-constructed) server/ws
    // objects and starts broadcasting. Call once from setup(), AFTER
    // WiFi.softAP()/WiFi.begin() have been kicked off.
    void begin(AsyncWebServer* server, AsyncWebSocket* ws);

    // Call once a second from loop() with the same DisplayData just built
    // for the LCD. Serializes it and pushes it to every connected
    // WebSocket client. Cheap no-op if nobody's connected.
    void pushState(const DisplayData& dd);

    // Call every loop() iteration (no timing gate needed -- it's a cheap
    // no-op when nothing new has been written). Forwards anything new in
    // WebSerialTee's ring buffer to the Console tab's WebSocket, giving a
    // near-real-time tail of Logger/menu output in the browser.
    void pumpLog();

    // Call every loop() iteration. Runs any console commands that arrived from
    // the web UI (Console tab, "clear faults" button). They are queued by the
    // web server's task and executed HERE, in loop(), because many commands
    // block for seconds (charger confirm/re-apply) or talk on the BMB serial
    // line, which the web server's task must not do -- blocking it trips its
    // watchdog, and it would collide with loop()'s own BMB traffic.
    void processQueuedCommands();

private:
    QueueHandle_t _cmdQueue = nullptr;   // char* lines, filled by the web task, drained by loop()
    void queueConsoleCommand(const String& line);
    AsyncWebServer* _server = nullptr;
    AsyncWebSocket* _ws     = nullptr;
    AsyncWebSocket  _wsLog{"/wslog"};
    uint32_t        _logCursor = 0;
    String _lastStateJson;   // so a client that connects between pushes still gets something immediately

    bool checkAuth(AsyncWebServerRequest* request);

    String buildStateJson(const DisplayData& dd);
    String buildSettingsJson();

    void onIndex(AsyncWebServerRequest* request);
    void onApiState(AsyncWebServerRequest* request);
    void onApiSettingsGet(AsyncWebServerRequest* request);
    void onApiSettingsPost(AsyncWebServerRequest* request, const String& body);
    void onApiChargerPost(AsyncWebServerRequest* request, const String& body);
    void onApiFaultsClear(AsyncWebServerRequest* request);
    void onApiReboot(AsyncWebServerRequest* request);

    void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len);
    void onLogWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len);
};

extern WebUIManager webUI;