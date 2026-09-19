// LED States
// Green flash = all packs found
// Purple flash = searching for BMBs
// Blue flash = wifi connected, normal operation

// --------------------- Includes ---------------------
#include <Arduino.h>
#include "Logger.h"
#include "SerialConsole.h"
#include "config.h"
#include "BMSModuleManager.h"
#include "SystemIO.h"
#include "Displaymanager.h"
#include "Chargernpb.h"
#include "Webui.h"
//#include "MQTTClient.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
//#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include "secrets.h"
//#include <LittleFS.h> // Will be used later to host the Web frontend to show stats
#include <Adafruit_NeoPixel.h> // To drive ws2812 LED
//#include "esp_adc_cal.h" // For calibration to improve readings via ADC - not used in this project.

// handleRoot()/handleUpdate() and the old inline route lambdas are gone --
// all HTTP/WebSocket routes now live in WebUIManager (WebUI.h/.cpp), wired
// up via webUI.begin(&server, &ws) in setup().

// ------------------------- Init variables -------------------------
// -- Define the RX and TX pins used to talk to the BMBs --
#define BMB_RX_PIN      16
#define BMB_TX_PIN      17
#define BMB_FAULT_PIN   4

// -- Charger CAN (MEAN WELL NPB-750-24 via SN65HVD230 transceiver) --
// Wiring on this build: ESP32 CAN RX = GPIO1, CAN TX = GPIO2.
// ChargerNPB::begin() takes (txPin, rxPin), so the call below passes
// GPIO_NUM_2 first (TX) then GPIO_NUM_1 (RX). On the transceiver, GPIO2 must
// land on CTX (driver input) and GPIO1 on CRX (receiver output) -- straight
// through, NOT crossed like UART.
#define CHARGER_CAN_TX_PIN  GPIO_NUM_2
#define CHARGER_CAN_RX_PIN  GPIO_NUM_1

// Fallback defaults used ONLY the very first boot, before anything has ever
// been saved to Preferences.
//
// Tuned for this pack: 6S Li-ion, 4.20V/cell max = 25.2V. 22.5A CC / 2.25A TC
// (10% taper cutoff) are well within the pack's actual capability (6s74p --
// ~0.3A/cell at full rated current) and just reflect the charger's own max
// output, not a pack-specific limit. RSTV (restart-to-charge threshold, only
// active if SYSTEM_CONFIG's RSTE bit is enabled -- see CHGRAWW/CHGOPINIT) is
// set just under the float voltage so the charger can auto-resume after real
// self-discharge without restarting on every tiny BMS balancing dip.
//
// IMPORTANT: CHARGER_INIT_VOLT/CHARGER_INIT_CURR are effectively historical --
// chargerVoltage/chargerCurrent are aliases of chargerCurveCV/chargerCurveCC
// (the registers that actually govern charging) and get overwritten by them
// during setup(), before first use. Kept in sync here purely so the initial
// global value isn't visibly wrong for the brief moment before setup() runs.
#define CHARGER_INIT_VOLT       25.2f
#define CHARGER_INIT_CURR       22.5f
#define CHARGER_INIT_CURVE_CC   22.5f
#define CHARGER_INIT_CURVE_CV   25.2f
#define CHARGER_INIT_CURVE_FV   24.6f
#define CHARGER_INIT_CURVE_TC   2.25f
#define CHARGER_INIT_RSTV       24.6f
#define CHARGER_INIT_CCTO       0
#define CHARGER_INIT_CVTO       0
#define CHARGER_INIT_FVTO       0

// Daily vs full-charge target voltages. Li-ion doesn't need to sit at 100%
// between uses -- Tesla's own daily-driving limit is ~80% SOC (4.00V/cell),
// reserving the top of the range for occasional full charges before a
// demanding day. 3.30V/cell is the pack's low-end floor (see VOLTLIMLO for
// the BMS-side cutoff that actually enforces this).
#define CHARGER_INIT_DAILY_V    24.0f   // 4.00V/cell x 6S -- everyday charge target (~80% SOC)
#define CHARGER_INIT_FULL_V     24.9f   // 4.15V/cell x 6S -- full-charge override target

// -- MQTT Auth configuration --
#define MQTT_USER SECRET_MQTT_USER // update this in secrets.h
#define MQTT_PASSWORD SECRET_MQTT_PASSWORD // update this in secrets.h
#define MQTT_CLIENT_NAME "BMSClient"

// -- Speed at which we talk to tesla BMBs --
// Possible settings are 631578,612500,617647,608695
#define BMS_BAUD        631578

// -- LED(s) configuration --
#define PIN             48    // GPIO to which the LED(s strip) is/are connected
#define NUMPIXELS       1    // Number of LEDs
#define BRIGHTNESS      15   // Adjust brightness (0-255)
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// --  Buzzer configuration --
#define BUZZER_PIN      21
#define CHIRP_ON_MS     80      // length of each chirp
#define CHIRP_GAP_MS    100     // gap between the two chirps in a pair
#define CHIRP_PAUSE_MS  1200    // pause after the second chirp before repeating

bool buzzerOn = false;

enum BuzzerState { BUZZ_IDLE, BUZZ_CHIRP1, BUZZ_GAP, BUZZ_CHIRP2, BUZZ_PAUSE };
BuzzerState buzzerState = BUZZ_IDLE;
uint32_t buzzerStateStart = 0;

// ------------------------- Global Vars -------------------------
Preferences preferences;
AsyncWebServer server(80);
// Catch-all DNS for the AP interface -- answers every lookup from an AP
// client with the AP's own IP, so http://<mdnsHostname> (or literally any
// hostname) resolves reliably without depending on mDNS/Bonjour at all.
// This is the same trick captive-portal Wi-Fi networks use. There's no
// internet routed through this AP anyway, so answering every query is
// harmless -- it just means anything you type while connected to the AP
// lands on this device.
DNSServer dnsServer;
#define DNS_PORT 53
AsyncWebSocket ws("/ws");
BMSModuleManager bms(&server);
EEPROMSettings settings;
SerialConsole console;
uint32_t lastUpdate1;
uint32_t lastUpdate2;
uint32_t lastUpdate3;
String bmsJson;
float balanceVoltage = 3.95f;
float balanceHyst = 0.007f;

// -- Charger setpoints -- persisted the same way as balanceVoltage/balanceHyst
// above (own Preferences keys, loaded at boot, updated live from the serial
// console). These are what actually get pushed to the charger at boot and
// whenever changed via CHGxxx commands, and what printMenu() displays live
// (mirroring how VOLTLIMHI etc. show the real settings.OverVSetpoint value).
float    chargerVoltage      = CHARGER_INIT_VOLT;       // live VOUT_SET
float    chargerCurrent      = CHARGER_INIT_CURR;       // live IOUT_SET
float    chargerCurveCC      = CHARGER_INIT_CURVE_CC;   // curve constant-current target
float    chargerCurveCV      = CHARGER_INIT_CURVE_CV;   // curve constant-voltage target
float    chargerCurveFV      = CHARGER_INIT_CURVE_FV;   // curve float-voltage target
float    chargerCurveTC      = CHARGER_INIT_CURVE_TC;   // curve taper-current cutoff
float    chargerRstVbat      = CHARGER_INIT_RSTV;       // auto-restart-charge voltage point
uint16_t chargerCCTimeoutMin = CHARGER_INIT_CCTO;       // 0 = disabled
uint16_t chargerCVTimeoutMin = CHARGER_INIT_CVTO;
uint16_t chargerFVTimeoutMin = CHARGER_INIT_FVTO;

// Daily-limit / full-charge-override system. chargerCurveCV (the register
// that actually governs boost/charge voltage) is now DERIVED from these --
// see applyChargeTargetVoltage() below -- rather than a fixed value. CHGCV/
// CHGV remain available for manual/ad-hoc testing, but get overwritten the
// next time this system re-asserts itself (boot, override toggle, or
// full-charge completion).
float chargerDailyTargetV       = CHARGER_INIT_DAILY_V;
float chargerFullTargetV        = CHARGER_INIT_FULL_V;
bool  chargerFullChargeOverride = false;  // true = charging toward chargerFullTargetV instead of chargerDailyTargetV

//const char* volt_str;
//WiFiClient espClient;
//PubSubClient client(espClient);
unsigned long rebootTime = 0;
int packsConfigured;
String systemName = "esp32-teslabms";
// -- Stuff below here needs to be configured in secrets.h --
String ftpServer = SECRET_FTP_SERVER_IP;
String ftpUser = SECRET_FTP_USER;
String ftpPassword = SECRET_FTP_PASSWORD;
String wifiSSID = SECRET_WIFI_SSID;
String wifiPassword = SECRET_WIFI_PASSWORD;
String apSSID = SECRET_AP_SSID;
String apPassword = SECRET_AP_PASSWORD;
// The friendly name you connect with instead of an IP -- http://<mdnsHostname>.local
// Only takes effect on next boot (mDNS responder is started once in setup()).
String mdnsHostname = "Lift";
String webUsername = SECRET_WEBUI_USER;
String webPassword = SECRET_WEBUI_PASS;
String mqttServerIP = SECRET_MQTT_SERVER_IP;
String mqtt_Topic = SECRET_MQTT_TOPIC;
// -- Stuff above here needs to be configured in secrets.h --

// The AP is ALWAYS up (this thing lives on an isolated AP most of its life --
// see WebUI.h). wifiSSID/wifiPassword are for the OPTIONAL home-network
// fallback only. wifiReconnectPending is how the web settings API asks
// loop() to apply a changed SSID/password -- set the flag and return
// immediately; the actual WiFi.begin()/status wait happens here in loop(),
// never inside an AsyncWebServer request handler (blocking there was almost
// certainly why the old UI would occasionally crash/hang on save).
bool wifiReconnectPending = false;
DisplayManager displayManager;  // Single global display instance
bool currentFaultState = false;
bool lastHwFaultState = false;   // tracks BMB_FAULT_PIN state for edge-triggered logging
bool chargerReady = false;       // true once charger.begin() succeeds at boot -- gates the charging page

// What we WANT the charger's output to be. This is the thing that survives
// until the user toggles 'o' or the fault-interlock/fault-clear logic below
// changes it -- it does NOT get silently overwritten by telemetry. Its
// purpose is to catch the charger drifting out of sync with our intent on
// its own (e.g. it auto-restarts its output after an AC power cycle while we
// wanted it OFF) -- enforceChargerDesiredState() below corrects that.
bool     desiredChargerOn = false;
uint32_t lastChargerCorrectionMs = 0;
#define CHARGER_CORRECTION_COOLDOWN_MS 3000  // don't hammer the bus every poll while mismatched

// Set by the WebUI's /api/charger handler when a curve-register change needs
// to be reapplied live (ChargerNPB::reapplyCurveNow() -- a confirmed off/on
// toggle, blocking up to ~8s). Same deferred-to-loop() pattern as
// wifiReconnectPending above and for the same reason: reapplyCurveNow() must
// NEVER be called from inside an AsyncWebServer request handler.
bool     chargerCurveReapplyPending = false;

// Fault history tracking -- purely observational, does not affect whether or
// how faults get cleared. Captures duration + time-since-cleared for the most
// recent fault event so momentary blips are visible even though the live
// FAULT/OK status is non-latching (the BMB's own fault registers may be
// sticky and require clearFaults() to actually release -- this tracker just
// reports what the combined fault signal did, whatever the underlying cause
// of it clearing was).
bool     faultActive = false;
uint32_t faultStartMillis = 0;
bool     hasFaultHistory = false;
uint32_t lastFaultDurationMs = 0;
uint32_t lastFaultClearedAtMillis = 0;
char     lastActiveFaultReason[40] = "";  // most recently *seen* active fault reason, refreshed every cycle while faulted
int      lastActiveFaultCount = 0;        // how many were active as of that last snapshot
char     lastClearedFaultReason[40] = ""; // what the fault actually was, frozen at the moment it cleared -- shown in history
int      lastClearedFaultCount = 0;

// Manual fault injection for testing -- triggered by the 't' serial console
// command. Lets you verify the buzzer, FAULT pill, and fault-history readout
// all work end-to-end without needing a real BMB fault to happen.
bool     testFaultOverride = false;
uint32_t testFaultUntilMillis = 0;

// ------------------ MQTT ------------------
String mqttServer;
const int mqttPort = 1883;
int websocketsPort = 5081;
int mqttRetryCount = 0;
const int maxMqttRetries = 3;
String mqttTopic = mqtt_Topic;
const char* mqttClientName = MQTT_CLIENT_NAME;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASSWORD;

// --- Function to manage LED behaviour ---
void setLED(uint8_t r, uint8_t g, uint8_t b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}

// --- Function for fast blue LED flash
void flashBlue(int flashes = 5) {
    for (int i = 0; i < flashes; i++) {
        setLED(0, 0, 255);
        delay(100);
        setLED(0, 0, 0);
        delay(100);
    }
}

// --- Function for fast purple LED flash
void flashPurple(int flashes = 5) {
    for (int i = 0; i < flashes; i++) {
        setLED(128, 0, 255);
        delay(100);
        setLED(0, 0, 0);
        delay(100);
    }
}

// --- Function for fast Green LED flash
void flashGreen(int flashes = 5) {
    for (int i = 0; i < flashes; i++) {
        setLED(0, 255, 0);
        delay(100);
        setLED(0, 0, 0);
        delay(100);
    }
}

// If the charger's actual output state doesn't match what we last commanded
// (desiredChargerOn), push it back into line. This catches the charger
// drifting on its own -- most notably auto-restarting its output after an AC
// power cycle while we wanted it left OFF. Deliberately non-blocking: a
// single setOutput() call, rate-limited by CHARGER_CORRECTION_COOLDOWN_MS so
// an unreachable charger doesn't get hammered every poll cycle. Call this
// after charger.poll() so charger.data() is fresh.
void enforceChargerDesiredState() {
    if (!chargerReady) return;
    const ChargerData& cd = charger.data();
    if (!cd.online) return;
    if (cd.outputOn == desiredChargerOn) return;

    if (millis() - lastChargerCorrectionMs < CHARGER_CORRECTION_COOLDOWN_MS) return;
    lastChargerCorrectionMs = millis();

    Logger::warn("Charger state mismatch: actual=%s desired=%s -- correcting",
                 cd.outputOn ? "ON" : "OFF", desiredChargerOn ? "ON" : "OFF");
    if (!charger.setOutput(desiredChargerOn))
        Logger::error("Charger correction command failed -- charger not responding, will retry");
}

// Writes whichever target (daily-limit or full-charge override) is
// currently selected into chargerCurveCV/CURVE_CV -- this is what actually
// decides the charger's charge voltage day to day. Also mirrors the same
// value into CURVE_FV: if the charger ever enters a float stage after
// tapering, it must never float ABOVE the target that was just reached (a
// separate, higher fixed float voltage would defeat the whole point of a
// daily limit) -- holding at the same voltage it already achieved is safe,
// anything higher isn't.
//
// Fast, non-blocking CAN writes only (safe to call from anywhere, including
// WebUI request handlers). Does NOT force a live reapply -- callers that
// need the change to take effect immediately should follow this with
// chargerCurveReapplyPending = true (deferred, WebUI-safe) or
// charger.reapplyCurveNow() directly (only from loop()/console context,
// never a request handler -- see the many other comments on this).
void applyChargeTargetVoltage() {
    chargerCurveCV = chargerFullChargeOverride ? chargerFullTargetV : chargerDailyTargetV;
    chargerVoltage = chargerCurveCV; // keep the CHGV/dashboard alias in sync
    chargerCurveFV = chargerCurveCV; // float, if entered, must never exceed the active target
    if (chargerReady) {
        charger.setCurveCV(chargerCurveCV);
        charger.setCurveFV(chargerCurveFV);
    }
}

// --- Attempt to connect to mqtt; fail and continue if it doesn't work after a few retries ---
// void connectMQTT() {
//     unsigned long startAttemptTime = millis();
//     const unsigned long timeout = 3000;
//     mqttRetryCount = 0; // Reset retry count
//     while (mqttRetryCount < maxMqttRetries && (millis() - startAttemptTime) < timeout) {
//         if (client.connect(
//         (systemName + "_" + mqttClientName).c_str(), mqttUser, mqttPassword)) {
//             mqttRetryCount = 0; // Reset on success
//             return;
//         } else {
//             Serial.print("Failed with state ");
//             Serial.println(client.state());
//             mqttRetryCount++;
//             delay(2000);
//         }
//     }
//     Serial.println("Unable to connect to MQTT broker. Proceeding without MQTT.");
// }

void loadSettings()
{
    Logger::console("Resetting to factory defaults");
    settings.version = EEPROM_VERSION;
    settings.checksum = 0;
    settings.canSpeed = 500000;
    settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
    // 4.20V/cell over-voltage fault ceiling -- deliberately ABOVE the 4.15V/cell
    // full-charge override target (CHARGER_INIT_FULL_V), so an intentional full
    // charge doesn't trip this fault right as it reaches the target. 3.30V/cell
    // under-voltage floor per the pack's actual low-end limit.
    settings.OverVSetpoint = 4.20f;
    settings.UnderVSetpoint = 3.30f;
    settings.OverTSetpoint = 65.0f;
    settings.UnderTSetpoint = -10.0f;
    settings.balanceVoltage = 3.95f;
    settings.balanceHyst = 0.007f;
    settings.logLevel = 1;
    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
}

/* - CAN bus code - not used - needs to be cleaned up in the future.
void initializeCAN()
{
    uint32_t id;
    CAN0.begin(settings.canSpeed);
    if (settings.batteryID < 0xF)
    {
        //Setup filter for direct access to our registered battery ID
        id = (0xBAul << 20) + (((uint32_t)settings.batteryID & 0xF) << 16);
        CAN0.setRXFilter(0, id, 0x1FFF0000ul, true);
        //Setup filter for request for all batteries to give summary data
        id = (0xBAul << 20) + (0xFul << 16);
        CAN0.setRXFilter(1, id, 0x1FFF0000ul, true);
    }
}
*/

void setup()
{
    // LCD RST (GPIO9) has no external pull-up -- drive it HIGH (deasserted,
    // not-in-reset) as the very first thing we do, before anything else,
    // to shrink the floating window between power-on and DisplayManager::
    // begin() -> _lcd.init() (which takes over this pin and does a proper
    // reset pulse anyway). This doesn't cover the ROM-bootloader-only
    // window before setup() runs, but that window is brief and harmless --
    // no valid SPI transaction (CS not asserted) can happen during it, so
    // spurious RST activity there is a no-op, not a fault.
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);

    preferences.begin("settings", true); // "settings" is the namespace

    // Load saved settings
    mqttServer = preferences.getString("mqttServer", mqttServerIP);
    systemName = preferences.getString("systemName");
    wifiSSID = preferences.getString("wifiSSID", wifiSSID);
    wifiPassword = preferences.getString("wifiPassword", wifiPassword);
    apSSID = preferences.getString("apSSID", apSSID);
    apPassword = preferences.getString("apPassword", apPassword);
    mdnsHostname = preferences.getString("mdnsHostname", mdnsHostname);
    balanceVoltage = preferences.getFloat("balanceVoltage", 3.95f);
    balanceHyst = preferences.getFloat("balanceHyst", 0.007f);
    packsConfigured = preferences.getInt("packsConfigured", DEFAULT_PACKS_CONFIGURED);
    ftpPassword = preferences.getString("ftpPassword", ftpPassword);
    ftpUser = preferences.getString("ftpUser", ftpUser);
    ftpServer = preferences.getString("ftpServer", ftpServer);
    webUsername = preferences.getString("webUsername", webUsername);
    webPassword = preferences.getString("webPassword", webPassword);
    preferences.end();

    // -- Initialise the LED and show red to indicate boot --
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show();
    setLED(255, 0, 0);  // Solid RED on boot

    /*
    if (!LittleFS.begin(true)) {
        Serial.println("An error occurred while mounting LittleFS");
        return;
    }
    */

    delay(2000);  //For easy debugging. It takes a few seconds for USB to come up properly
    SERIALCONSOLE.begin(115200);
    SERIALCONSOLE.println("Starting up!");

    // The AP is the primary interface -- this device spends nearly all its
    // life off any home network, controlled entirely over its own AP. It
    // comes up unconditionally, every boot, regardless of what happens with
    // the optional home-Wi-Fi (STA) join below.
    //
    // Explicit, non-default subnet is deliberate: the ESP32 Arduino core's
    // default softAP subnet is 192.168.4.0/24, which collides with plenty
    // of home routers' own default range (this bit us directly -- home
    // Wi-Fi handed the STA interface 192.168.4.46, same /24 as the AP).
    // With WIFI_AP_STA on the same subnet on both interfaces, the ESP32's
    // routing gets ambiguous about which interface to send return traffic
    // out on -- requests can arrive fine but replies silently go out the
    // wrong interface and never make it back. 192.168.44.0/24 is far less
    // likely to ever collide with a home network's own range.
    WiFi.mode(WIFI_AP_STA);
    IPAddress apIP(192, 168, 44, 1);
    IPAddress apGateway(192, 168, 44, 1);
    IPAddress apSubnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apGateway, apSubnet);
#if AP_REQUIRE_PASSWORD
    WiFi.softAP(apSSID.c_str(), apPassword.c_str());
#else
    WiFi.softAP(apSSID.c_str()); // no password argument = open network
#endif
    Serial.println("AP '" + apSSID + "' started. IP address: " + WiFi.softAPIP().toString());

    // Hostname resolution for AP clients, answering with the AP's own IP.
    // Deliberately scoped to ONLY the configured hostname now, not "*" --
    // a wildcard answers every DNS query, which means every phone/laptop's
    // automatic background traffic (Apple's captive-portal check, Android's
    // connectivity check, random apps probing random hosts) all lands
    // directly on this device's web server the moment it joins the AP.
    // That extra concurrent/unexpected traffic is a very likely contributor
    // to a known, long-standing race-condition crash in the
    // ESPAsyncWebServer-esphome library itself (LoadProhibited in
    // _parseLine()/_onData() -- documented in multiple upstream GitHub
    // issues, not something introduced here). Narrowing this is cheap
    // insurance even though it doesn't fix the library bug directly.
    // NOTE: deliberately NOT ".local" -- per RFC 6762, Apple devices treat
    // .local as reserved for mDNS/Bonjour and will ALWAYS resolve it via
    // multicast DNS, never through the regular DNS server, no matter what
    // the AP hands out via DHCP. That's exactly why this worked instantly
    // on a Windows laptop but never on an iPhone. Any other suffix (.iot,
    // .home, .lan, whatever) is ordinary as far as every OS's resolver is
    // concerned and works identically everywhere.
    String dnsMatchName = mdnsHostname + ".iot";
    dnsServer.start(DNS_PORT, dnsMatchName, apIP);
    Serial.println("DNS started on the AP -- http://" + dnsMatchName + " resolves here while connected to '" + apSSID + "'.");

    // Optional: also try to join a home network, bounded so it can never
    // hang setup() (and never runs again from inside a request handler --
    // see wifiReconnectPending in loop()).
    if (wifiSSID.length() > 0) {
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        Serial.println("Attempting to also join Wi-Fi SSID: " + wifiSSID);

        unsigned long startAttemptTime = millis();
        const unsigned long timeout = 10000; // 10 seconds -- keep boot snappy, AP is already up either way

        while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime) < timeout) {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Wi-Fi connected successfully.");
            Serial.println("IP address: " + WiFi.localIP().toString());
        } else {
            Serial.println("Wi-Fi join failed/timed out -- continuing on AP only.");
        }
    }

    // mDNS is initialized down near ArduinoOTA.begin() (it starts mDNS
    // internally using ArduinoOTA.setHostname()) -- we just add our own
    // HTTP service to that same responder afterward via MDNS.addService(),
    // rather than calling MDNS.begin() ourselves up here. Calling
    // MDNS.begin() a second time with a different hostname than OTA's
    // internal one is what caused the earlier crash/UDP flood.

    SERIAL.begin(BMS_BAUD, SERIAL_8N1, BMB_RX_PIN, BMB_TX_PIN);
    SERIALCONSOLE.println("Started serial interface to BMS.");
    pinMode(BMB_FAULT_PIN, INPUT_PULLUP); // Setup FAULT hardware line as input.
    // INPUT_PULLUP (not plain INPUT) because the fault line is active-low and
    // open-drain from the BMB side -- it needs a pull-up to 3.3V so it reads
    // a clean HIGH when no fault is asserted. A series resistor elsewhere in
    // the line doesn't provide this; it just limits current. The ESP32's
    // internal ~45k pull-up is safe to enable even if an external pull-up
    // already exists -- it only adds a weak parallel path to 3.3V and won't
    // stop the line from being pulled LOW when a module asserts a fault.
    Serial.println("Load EEPROM");
    loadSettings();
    Serial.println("Load custom settings");
    preferences.begin("settings", true); // "settings" is the namespace
//  settings.balanceVoltage = preferences.getFloat("balanceVoltage"), 3.95f);
//  settings.balanceHyst = preferences.getFloat("balanceHyst", 0.007f);
    settings.balanceVoltage = balanceVoltage;
    settings.balanceHyst    = balanceHyst;
    // Load the remaining serial-configurable settings too, so anything set via
    // the console (VOLTLIMHI, VOLTLIMLO, TEMPLIMHI, TEMPLIMLO, CANSPEED,
    // BATTERYID, LOGLEVEL) survives a reboot even with no WiFi/WebUI access.
    // Falls back to whatever loadSettings() already put in place if no saved
    // value exists yet.
    settings.OverVSetpoint  = preferences.getFloat("overVSetpoint",  settings.OverVSetpoint);
    settings.UnderVSetpoint = preferences.getFloat("underVSetpoint", settings.UnderVSetpoint);
    settings.OverTSetpoint  = preferences.getFloat("overTSetpoint",  settings.OverTSetpoint);
    settings.UnderTSetpoint = preferences.getFloat("underTSetpoint", settings.UnderTSetpoint);
    settings.canSpeed       = preferences.getUInt("canSpeed",        settings.canSpeed);
    settings.batteryID      = preferences.getUChar("batteryID",      settings.batteryID);
    settings.logLevel       = preferences.getUChar("logLevel",       settings.logLevel);
    Logger::setLoglevel((Logger::LogLevel)settings.logLevel); // re-apply in case a saved value overrode loadSettings()'s default

    // Charger setpoints -- own Preferences keys (don't collide with anything
    // in EEPROMSettings). Falls back to the CHARGER_INIT_* defaults above if
    // nothing has ever been saved (first boot).
    //
    // chargerVoltage/chargerCurrent (CHGV/CHGI) and chargerCurveCV/chargerCurveCC
    // (CHGCV/CHGCC) are the SAME physical setpoints -- CURVE_CV/CURVE_CC are
    // the only registers that actually govern output while the charger is
    // running its curve/battery-charging profile (which is always, in this
    // application). The curve values are treated as authoritative here in
    // case the two Preferences keys ever drifted apart from an older build.
    chargerCurveCC      = preferences.getFloat("chgCurveCC",  chargerCurveCC);
    chargerCurveCV      = preferences.getFloat("chgCurveCV",  chargerCurveCV);
    chargerVoltage      = chargerCurveCV;
    chargerCurrent      = chargerCurveCC;
    chargerCurveFV      = preferences.getFloat("chgCurveFV",  chargerCurveFV);
    chargerCurveTC      = preferences.getFloat("chgCurveTC",  chargerCurveTC);
    chargerRstVbat      = preferences.getFloat("chgRstVbat",  chargerRstVbat);
    chargerCCTimeoutMin = preferences.getUShort("chgCCTmout", chargerCCTimeoutMin);
    chargerCVTimeoutMin = preferences.getUShort("chgCVTmout", chargerCVTimeoutMin);
    chargerFVTimeoutMin = preferences.getUShort("chgFVTmout", chargerFVTimeoutMin);

    // Daily-limit / full-charge-override system -- see applyChargeTargetVoltage().
    // Loaded after chargerCurveCV/CV above; applyChargeTargetVoltage() (called
    // once the charger comes up, below) overwrites chargerCurveCV/CurveFV with
    // whichever of these is active, so those earlier loads are just a harmless
    // fallback for the brief window before that runs.
    chargerDailyTargetV       = preferences.getFloat("chgDailyV", chargerDailyTargetV);
    chargerFullTargetV        = preferences.getFloat("chgFullV",  chargerFullTargetV);
    chargerFullChargeOverride = preferences.getBool("chgFullOvr", chargerFullChargeOverride);

    preferences.end();

    Serial.println("Loaded settings from flash (NVS):");
    Serial.printf("  CANSPEED:   %lu\r\n", (unsigned long)settings.canSpeed);
    Serial.printf("  BATTERYID:  %u\r\n",  settings.batteryID);
    Serial.printf("  LOGLEVEL:   %u\r\n",  settings.logLevel);
    Serial.printf("  VOLTLIMHI:  %.3f\r\n", settings.OverVSetpoint);
    Serial.printf("  VOLTLIMLO:  %.3f\r\n", settings.UnderVSetpoint);
    Serial.printf("  TEMPLIMHI:  %.3f\r\n", settings.OverTSetpoint);
    Serial.printf("  TEMPLIMLO:  %.3f\r\n", settings.UnderTSetpoint);
    Serial.printf("  BALVOLT:    %.3f\r\n", settings.balanceVoltage);
    Serial.printf("  BALHYST:    %.3f\r\n", settings.balanceHyst);
    Serial.printf("  CHGV:       %.2f\r\n", chargerVoltage);
    Serial.printf("  CHGI:       %.2f\r\n", chargerCurrent);
    Serial.printf("  CHGCC:      %.2f\r\n", chargerCurveCC);
    Serial.printf("  CHGCV:      %.2f\r\n", chargerCurveCV);
    Serial.printf("  CHGFV:      %.2f\r\n", chargerCurveFV);
    Serial.printf("  CHGTC:      %.2f\r\n", chargerCurveTC);
    Serial.printf("  CHGRSTV:    %.2f\r\n", chargerRstVbat);
    Serial.printf("  CHGCCTO:    %u\r\n",   chargerCCTimeoutMin);
    Serial.printf("  CHGCVTO:    %u\r\n",   chargerCVTimeoutMin);
    Serial.printf("  CHGFVTO:    %u\r\n",   chargerFVTimeoutMin);
//  Serial.println("Initialize CAN");
//  initializeCAN();
    Serial.println("System IO setup");
    // systemIO.setup();  // disabled -- conflicts with LCD pins (10-14); re-enable once DOUT is remapped to 21-24 pool for load-disconnect relay work
    Serial.println("Done.");

    // -- Charger CAN bring-up --
    // Per the MEAN WELL NPB/NPP User Manual, floating A0/A1 address pins
    // read as logic 1/1 = Device address 3 -- charger.begin() defaults to
    // that address (matches the manual's own worked CAN examples, which all
    // use ID 0xC0103). If you later ground A0/A1 to force a different
    // address, pass it explicitly: charger.begin(tx, rx, 0x00) etc.
    //
    // Starts the TWAI peripheral and, if it comes up, pushes every saved
    // charger setpoint (live V/I plus the curve/timeout registers) so the
    // charger picks up right where you left it via the serial console --
    // same idea as loading balanceVoltage/settings.OverVSetpoint above. If
    // the bus fails to start (bad wiring, transceiver unpowered) we log and
    // continue -- the BMS still runs fine without the charger.
    Serial.println("Starting charger CAN...");
    //ChargerNPB::sniffBus(CHARGER_CAN_TX_PIN, CHARGER_CAN_RX_PIN, 8000);
    if (charger.begin(CHARGER_CAN_TX_PIN, CHARGER_CAN_RX_PIN)) {
        chargerReady = true;
        // NOTE: no separate VOUT_SET/IOUT_SET writes here -- those registers
        // are ignored while the charger runs its curve/battery-charging
        // profile (always, in this application). CURVE_CC/CURVE_CV below are
        // what actually govern output current/voltage; chargerVoltage/
        // chargerCurrent are just the CHGV/CHGI aliases for these same values.
        charger.setCurveCC(chargerCurveCC);
        applyChargeTargetVoltage(); // sets/pushes CURVE_CV and CURVE_FV from the daily/full-charge target system
        charger.setCurveTC(chargerCurveTC);
        charger.setChgRstVbat(chargerRstVbat);
        charger.setCurveCCTimeoutMinutes(chargerCCTimeoutMin);
        charger.setCurveCVTimeoutMinutes(chargerCVTimeoutMin);
        charger.setCurveFVTimeoutMinutes(chargerFVTimeoutMin);
        charger.setOutput(false);
        desiredChargerOn = false; // baseline intent from boot -- output stays OFF until 'o' is pressed (or a fault-clear turns it back on); enforceChargerDesiredState() keeps this enforced
        Serial.printf("Charger CAN up. Applied saved setpoints: V=%.2f I=%.2f CC=%.2f CV=%.2f FV=%.2f TC=%.2f RSTV=%.2f CCTO=%u CVTO=%u FVTO=%u, output OFF (boot default).\r\n",
                      chargerVoltage, chargerCurrent, chargerCurveCC, chargerCurveCV,
                      chargerCurveFV, chargerCurveTC, chargerRstVbat,
                      chargerCCTimeoutMin, chargerCVTimeoutMin, chargerFVTimeoutMin);

        // Immediate proof-of-life readout -- confirms real two-way comms
        // right now rather than waiting on the 3s poll loop and inferring
        // success from the absence of "no reply" warnings.
        float v, i, t;
        uint16_t fault, chgStat;
        bool gotV = charger.readVoltage(v);
        bool gotI = charger.readCurrent(i);
        bool gotT = charger.readTemp(t);
        bool gotF = charger.readFaultStatus(fault);
        bool gotC = charger.readChgStatus(chgStat);
        if (gotV || gotI || gotT || gotF || gotC) {
            Serial.println("Charger CAN: got real replies --");
            if (gotV) Serial.printf("  READ_VOUT:  %.2f V\r\n", v);
            if (gotI) Serial.printf("  READ_IOUT:  %.2f A\r\n", i);
            if (gotT) Serial.printf("  READ_TEMP:  %.1f C\r\n", t);
            if (gotF) Serial.printf("  FAULT_STATUS: 0x%04X (%s)\r\n", fault, ChargerNPB::faultToString(fault).c_str());
            if (gotC) Serial.printf("  CHG_STATUS:   0x%04X\r\n", chgStat);
        } else {
            Serial.println("Charger CAN: still no replies to individual reads.");
        }
    } else {
        Serial.println("Charger CAN failed to start -- continuing without charger.");
    }

    //Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Initialise LCD display before BMS so we can show status during board search
    Serial.println("Starting display...");
    displayManager.begin();
    bms.setDisplay(&displayManager);
    Serial.println("Display started.");

    Serial.println("Finding BMS Boards...");
    bms.findBoards();
    Serial.println("Done.");
    Serial.println("Renumbering board IDs...");
    bms.renumberBoardIDs();

    // This block is for printing info at bootup
    if (numFoundModules < packsConfigured) {
        Serial.println("Found " + String(numFoundModules) + " out of " + String(packsConfigured) + " packs. Restarting search.");
        Serial.println("BMB RX / TX pins are set to " + String(BMB_RX_PIN) + " / " + String(BMB_TX_PIN));
    } else {
        Serial.println("Found all " + String(numFoundModules) + " packs! Search ended.");
    }

    //Logger::setLoglevel(Logger::Debug);

    lastUpdate1 = 0;
    lastUpdate2 = 0;
    lastUpdate3 = 0;

    Serial.println("BMS clear faults");
    bms.clearFaults();
    Serial.println("End of setup");
    Serial.println("Send ? line to get help. d to get detailed updates, p to get summary updates.");
    Serial.printf("Loaded balanceVoltage: %.2f\r\n", settings.balanceVoltage);
    Serial.printf("Loaded balanceHyst: %.3f\r\n", settings.balanceHyst);
    delay(1000);

    // -- Connect to MQTT broker --
    // client.setBufferSize(512);
    // client.setServer(mqttServer.c_str(), mqttPort);
    // client.setCallback(callback);
    // if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    //     connectMQTT();
    // }

    // All HTTP/WebSocket routes (the new single-page app, live telemetry
    // push, settings/charger JSON APIs, firmware upload) are registered here.
    webUI.begin(&server, &ws);

    // Start the server
    Serial.println("Starting HTTP server...");
    server.begin();
    Serial.println("HTTP server started.");

    // OTA Setup
    // setHostname() BEFORE begin() -- ArduinoOTA.begin() starts mDNS
    // internally using whatever hostname was set here. That's the ONLY
    // MDNS.begin() call anywhere in this program now; the earlier crash
    // came from calling MDNS.begin() ourselves separately (with a
    // different hostname than OTA's default), which fought with OTA's own
    // internal init. Setting the hostname here and letting OTA own the
    // single init, then just adding our own service afterward, is the
    // supported way to combine the two.
    ArduinoOTA.setHostname(mdnsHostname.c_str());
    ArduinoOTA
        .onStart([]() {
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("End");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress * 100) / total);
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    Serial.println("OTA Ready");

    // Piggyback our own HTTP service onto the mDNS responder ArduinoOTA
    // just started above -- NOT a second .begin() call, just registering
    // an additional service record on the already-running responder.
    if (MDNS.addService("http", "tcp", 80)) {
        Serial.println("mDNS: http://" + mdnsHostname + ".local should now resolve on mDNS-capable clients (iOS/macOS included).");
    } else {
        Serial.println("mDNS: addService(http) failed -- .local name may not resolve, but the .iot DNS fallback and IP address still work regardless.");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("IP address: " + WiFi.localIP().toString());
    } else {
        Serial.println("AP IP address: " + WiFi.softAPIP().toString());
    }
}

/*
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}
*/

//Buzzer
void updateBuzzer(bool active) {
    uint32_t now = millis();

    if (!active) {
        if (buzzerOn) { digitalWrite(BUZZER_PIN, LOW); buzzerOn = false; }
        buzzerState = BUZZ_IDLE; // reset so it always starts a fresh double-chirp next time
        return;
    }

    switch (buzzerState) {
        case BUZZ_IDLE:
            digitalWrite(BUZZER_PIN, HIGH);
            buzzerOn = true;
            buzzerState = BUZZ_CHIRP1;
            buzzerStateStart = now;
            break;
        case BUZZ_CHIRP1:
            if (now - buzzerStateStart >= CHIRP_ON_MS) {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerOn = false;
                buzzerState = BUZZ_GAP;
                buzzerStateStart = now;
            }
            break;
        case BUZZ_GAP:
            if (now - buzzerStateStart >= CHIRP_GAP_MS) {
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerOn = true;
                buzzerState = BUZZ_CHIRP2;
                buzzerStateStart = now;
            }
            break;
        case BUZZ_CHIRP2:
            if (now - buzzerStateStart >= CHIRP_ON_MS) {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerOn = false;
                buzzerState = BUZZ_PAUSE;
                buzzerStateStart = now;
            }
            break;
        case BUZZ_PAUSE:
            if (now - buzzerStateStart >= CHIRP_PAUSE_MS) {
                buzzerState = BUZZ_IDLE; // triggers the next chirp1 immediately on the next call
            }
            break;
    }
}

void loop()
{
    updateBuzzer(currentFaultState);

    dnsServer.processNextRequest(); // AP catch-all DNS -- cheap, must be polled every iteration
    console.loop(); // For interacting with the debug menu over serial
    webUI.pumpLog(); // Mirror new SERIALCONSOLE output to the web Console tab -- cheap no-op if nothing new / nobody's watching

    // 1-second tasks
    if (millis() - lastUpdate3 >= 1000) {
        lastUpdate3 = millis();
        // Manage the LED state based on wi-fi connection
        if (WiFi.status() != WL_CONNECTED) {
            setLED(255, 70, 0);  // Solid Yellow - no Wi-Fi
        } else {
        }

        // Hardware fault line is active-low (see BMB_FAULT_PIN pinMode comment
        // in setup()). Log only on state changes to avoid spamming the console
        // every second while a fault is held active. Reported into the same
        // fault registry BMSModuleManager uses internally, so it shows up
        // alongside register/limit faults on the fault page automatically.
        bool hwFault = (digitalRead(BMB_FAULT_PIN) == LOW);
        if (hwFault != lastHwFaultState) {
            if (hwFault) Logger::error("BMB hardware FAULT line asserted (GPIO%d LOW)", BMB_FAULT_PIN);
            else Logger::info("BMB hardware FAULT line cleared (GPIO%d HIGH)", BMB_FAULT_PIN);
            lastHwFaultState = hwFault;
        }
        if (hwFault) {
            char reason[40];
            snprintf(reason, sizeof(reason), "HARDWARE FAULT LINE (GPIO%d)", BMB_FAULT_PIN);
            bms.reportFault("HWPIN", reason);
        } else {
            bms.clearFaultById("HWPIN");
        }

        // Expire the manual test fault after its hold time, if active.
        if (testFaultOverride && (int32_t)(millis() - testFaultUntilMillis) >= 0) {
            testFaultOverride = false;
            bms.clearFaultById("TESTFLT");
            Logger::info("Test fault injection ended");
        }
        if (testFaultOverride) {
            bms.reportFault("TESTFLT", "TEST FAULT (serial 't' cmd)");
        }

        DisplayData dd;
        bms.buildDisplayData(dd); // dd.isFaulted and the fault list are derived live from the registry above
        currentFaultState = dd.isFaulted;

        // Charger snapshot for the charging page. charger.data() just
        // returns the ChargerNPB's last poll() result (refreshed every 3s in
        // the 3-second task block below) -- no extra CAN traffic here.
        {
            const ChargerData& cd = charger.data();
            dd.chargerPresent      = chargerReady;
            dd.chargerOnline       = cd.online;
            dd.chargerOutputOn     = cd.outputOn;
            dd.chargerVout         = cd.vout;
            dd.chargerIout         = cd.iout;
            dd.chargerTemp         = cd.temp;
            dd.chargerVoltSetpoint = chargerVoltage;
            dd.chargerCurrSetpoint = chargerCurrent;
            dd.chargerChgStatusRaw = cd.chgStatus;
            dd.chargerFaulted      = ChargerNPB::isFaulted(cd.faultRaw);
            String cfs = ChargerNPB::faultToString(cd.faultRaw);
            strncpy(dd.chargerFaultStr, cfs.c_str(), sizeof(dd.chargerFaultStr) - 1);
            dd.chargerFaultStr[sizeof(dd.chargerFaultStr) - 1] = '\0';
            dd.chargerLastRxAgoMs = cd.online ? (millis() - cd.lastRxMs) : 0xFFFFFFFF;
        }

        // Charger safety interlock -- keep the charger's output OFF for the
        // ENTIRE duration of any active BMS fault, not just its leading
        // edge. This deliberately runs every cycle rather than only on the
        // dd.isFaulted transition: setOutput() only flips its local
        // outputOn flag on a successful CAN write, so if that first OFF
        // command gets lost (bus glitch, charger momentarily unresponsive),
        // retrying here every second means the charger doesn't sit there
        // charging, unsupervised, for the rest of the fault. This is a
        // second, independent layer on top of any hardware interlock (e.g.
        // gating the Remote ON/OFF pins with the kill switch) -- it only
        // helps while the ESP32 itself is alive and running.
        if (chargerReady && dd.isFaulted) {
            desiredChargerOn = false; // for the ENTIRE duration of the fault, not just the edge
            if (charger.data().outputOn) {
                if (charger.setOutput(false))
                    Logger::error("FAULT ACTIVE: charger output commanded OFF");
                else
                    Logger::error("FAULT ACTIVE: charger output OFF command FAILED -- charger not responding, will retry next cycle");
            }
        }

        // Fault history edge detection -- purely observational (see comment
        // on the globals above). This does not clear anything; it just
        // records what the combined fault signal did.
        if (dd.isFaulted && dd.activeFaultCount > 0) {
            // Keep a running snapshot of what's currently active, refreshed
            // every cycle, so whichever fault was present right before
            // everything cleared is still available to show afterward.
            strncpy(lastActiveFaultReason, dd.faultReasons[0], sizeof(lastActiveFaultReason) - 1);
            lastActiveFaultReason[sizeof(lastActiveFaultReason) - 1] = '\0';
            lastActiveFaultCount = dd.activeFaultCount;
        }

        if (dd.isFaulted && !faultActive) {
            faultActive = true;
            faultStartMillis = millis();
        } else if (!dd.isFaulted && faultActive) {
            faultActive = false;
            lastFaultDurationMs = millis() - faultStartMillis;
            lastFaultClearedAtMillis = millis();
            hasFaultHistory = true;
            strncpy(lastClearedFaultReason, lastActiveFaultReason, sizeof(lastClearedFaultReason) - 1);
            lastClearedFaultReason[sizeof(lastClearedFaultReason) - 1] = '\0';
            lastClearedFaultCount = lastActiveFaultCount;
            Logger::info("Fault cleared after %lu ms: %s", (unsigned long)lastFaultDurationMs, lastClearedFaultReason);

            // Fault's gone -- resume charging automatically (this system is
            // meant to run unattended). Re-assert CURVE_CC/CURVE_CV (the
            // registers that actually govern output during curve/battery
            // charging) before turning back on -- the fault's OFF period and
            // this ON command together ARE the remote-toggle event curve
            // registers need to latch in, so this is the right moment to
            // make sure they're current.
            if (chargerReady) {
                desiredChargerOn = true;
                charger.setCurveCC(chargerCurveCC);
                charger.setCurveCV(chargerCurveCV);
                if (charger.setOutput(true))
                    Logger::info("Fault cleared -- charger output commanded back ON");
                else
                    Logger::error("Fault cleared -- charger output ON command FAILED -- charger not responding");
            }
        }

        dd.hasFaultHistory          = hasFaultHistory;
        dd.lastFaultDurationMs      = lastFaultDurationMs;
        dd.secondsSinceFaultCleared = hasFaultHistory ? (millis() - lastFaultClearedAtMillis) / 1000 : 0;
        strncpy(dd.lastFaultReason, lastClearedFaultReason, sizeof(dd.lastFaultReason) - 1);
        dd.lastFaultReason[sizeof(dd.lastFaultReason) - 1] = '\0';
        dd.lastFaultCount = lastClearedFaultCount;

        displayManager.update(dd);
        webUI.pushState(dd);   // same DisplayData the LCD just used -- one source of truth for both

        // Apply a Wi-Fi credential change requested via the settings API.
        // Deliberately done here (not inside the request handler) and
        // deliberately non-blocking -- we kick off the join and let the
        // normal WiFi.status() checks elsewhere (LED, dashboard) reflect
        // however it turns out. The AP is untouched either way.
        if (wifiReconnectPending) {
            wifiReconnectPending = false;
            Serial.println("Applying new Wi-Fi credentials: " + wifiSSID);
            WiFi.disconnect();
            if (wifiSSID.length() > 0) {
                WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
            }
        }

        // Apply a curve-register change requested via the WebUI's /api/charger
        // handler. Deliberately done here, not inside the request handler --
        // reapplyCurveNow() blocks for up to ~8s (confirmed off/on toggle),
        // and blocking an AsyncWebServer handler is the same mistake that
        // caused the old UI's occasional crash/hang on save (see
        // wifiReconnectPending above). Blocking loop() itself briefly is
        // acceptable -- the console's 'o' command already does the same via
        // setOutputConfirmed().
        if (chargerCurveReapplyPending) {
            chargerCurveReapplyPending = false;
            if (chargerReady && desiredChargerOn && !currentFaultState) {
                Serial.println("Reapplying charger curve settings live...");
                if (charger.reapplyCurveNow())
                    Logger::info("Charger curve settings reapplied live");
                else
                    Logger::error("Charger curve reapply failed to confirm -- charger not responding");
            }
        }

    }


    // 3-second tasks
    if (millis() - lastUpdate1 >= 3000) {
        lastUpdate1 = millis();

        if (WiFi.status() == WL_CONNECTED) {
            flashBlue(3);

            // if (!client.connected()) {
            //     connectMQTT();
            // }

            bms.getAllVoltTemp();
            bms.balanceCells();
            charger.poll();   // refresh charger V/I/temp/fault snapshot
            enforceChargerDesiredState(); // catch charger drifting from our commanded state (e.g. AC-cycle auto-restart)
            //String volt_str = bms.csvData();
            //client.publish(mqttTopic, volt_str.c_str());
        } else {
            // Still poll BMS when Wi-Fi is absent so the display stays live
            bms.getAllVoltTemp();
            bms.balanceCells();
            charger.poll();   // refresh charger V/I/temp/fault snapshot
            enforceChargerDesiredState(); // catch charger drifting from our commanded state (e.g. AC-cycle auto-restart)
        }

        // Full-charge override auto-revert: once the charger itself reports
        // FULLM (fully charged) while we're in override mode, drop back to
        // the daily target so the NEXT charge cycle (tonight, or whenever it
        // next restarts) aims for the daily limit again instead of staying
        // parked at the full-charge target indefinitely.
        if (chargerFullChargeOverride) {
            const ChargerData& cd = charger.data();
            if (cd.online && (cd.chgStatus & NPB_CHG_FULLM)) {
                chargerFullChargeOverride = false;
                preferences.begin("settings", false);
                preferences.putBool("chgFullOvr", false);
                preferences.end();
                applyChargeTargetVoltage();
                Logger::info("Full charge complete -- reverting to daily charge target (%.2fV)", chargerDailyTargetV);
                if (chargerReady && desiredChargerOn && !currentFaultState) {
                    if (!charger.reapplyCurveNow())
                        Logger::error("Daily-target reapply after full charge failed to confirm -- charger not responding");
                }
            }
        }
    }

    ArduinoOTA.handle(); // Handle OTA events

    // 10-second tasks
    if (millis() - lastUpdate2 >= 10000) {
        lastUpdate2 = millis();

        // -- debug statistics - keep commented if not needed to prevent noise in serial log --
        /*
        printChipTemp();
        Serial.println("");
        Serial.print("Free heap: ");
        Serial.print(ESP.getFreeHeap() / 1024);
        Serial.println(" kB");
        */

        if (WiFi.status() == WL_CONNECTED) {
            flashBlue(3);

            // if (numFoundModules == packsConfigured) {
            //     bms.publishIndividualData(client, "homeassistant/sensor/bms/", systemName);
            //     bmsJson = bms.buildJsonData();
            //     bms.sendBatteryStats(systemName, ftpServer, ftpUser, ftpPassword, bmsJson);
            //     bms.broadcastBatteryStats(&ws, bmsJson);
            // }
        }

        if (numFoundModules < packsConfigured) {
            bms.findBoards();
            bms.renumberBoardIDs();
            if (numFoundModules < packsConfigured) {
                Serial.println("Found " + String(numFoundModules) + " out of " + String(packsConfigured) + " packs. Restarting search.");
                Serial.println("Check connections - BMB RX / TX pins are set to " + String(BMB_RX_PIN) + " / " + String(BMB_TX_PIN));
                flashPurple(3);
            } else {
                Serial.println("Found all " + String(numFoundModules) + " packs! Search ended.");
                flashGreen(3);
            }
        }
    }
}