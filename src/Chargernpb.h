#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  ChargerNPB — CANBus control for MEAN WELL NPB-750-24
// ─────────────────────────────────────────────────────────────────────────────
//
//  Hardware:
//    ESP32-S3 built-in TWAI (CAN) controller  ->  Waveshare SN65HVD230
//    transceiver  ->  NPB-750 14-pin control connector:
//        Pin 11 = CANH   Pin 12 = CANL   Pin 9/10 = GND-AUX
//
//  Transceiver wiring (SN65HVD230):
//        ESP32 TWAI_TX  ->  CTX (D pin)
//        ESP32 TWAI_RX  ->  CRX (R pin)
//        3V3            ->  VCC   (the SN65HVD230 is a 3.3V part)
//        GND            ->  GND   (common with charger GND-AUX)
//        CANH/CANL      ->  charger CANH (pin11) / CANL (pin12)
//
//  Protocol (per MEAN WELL NPB/NPP series User Manual, Chapter 6):
//    CAN 2.0B, 250 kbit/s, Extended 29-bit identifier frames.
//    Message ID = 0x000C0000 | (dir << 8) | chargerAddr
//      dir = 0x01 for Controller -> Charger (our TX)
//      dir = 0x00 for Charger -> Controller (our RX)
//      chargerAddr = 0x00-0x03, set by the charger's A0/A1 pins (Pin 1/2),
//        referenced to GND(Signal) Pin 4:
//          A0/A1 Open  = logic 1
//          A0/A1 Short (to GND signal) = logic 0
//        Device 0: A1=0 A0=0   Device 1: A1=0 A0=1
//        Device 2: A1=1 A0=0   Device 3: A1=1 A0=1
//      NOTE: with A0/A1 left unconnected (floating), both read logic 1,
//      which is Device 3 -- this matches the manual's own worked CAN
//      examples, which all use ID 0xC0103 (address 3).
//    Payload:
//      Read  request: 2 bytes  = [cmd_lo, cmd_hi]
//      Write command: 4+ bytes = [cmd_lo, cmd_hi, data_lo, data_hi, ...]
//      Reply         : [cmd_lo, cmd_hi, data_lo, data_hi, ...] (echoes cmd)
//    Values are little-endian, scaled per the datasheet command table
//    (VOUT/IOUT F=0.01, TEMP F=0.1).
//
//  IMPORTANT (safety / setup):
//    * The charger must be in AUTO-RANGING mode for CAN voltage/current
//      setpoints to take effect (all DIP OFF, do the ON->OFF sequence within
//      15 s, pins 7&8 jumpered). Auto-ranging is "for lithium batteries with
//      BMS only" per MEAN WELL — which is exactly this ESS application.
//    * Do NOT command values outside the NPB-750-24 hardware ranges. This
//      driver clamps every setpoint, but verify against your pack.
// ─────────────────────────────────────────────────────────────────────────────

// ── NPB-750-24 hardware limits (from datasheet page 2) ──────────────────────
//   Charge voltage programming range : 21.0 – 42.0 V
//   Output voltage operating range   : 21.5 – 26.0 V   (auto-ranging window)
//   Max output current (CC)          : 22.5 A
//   Default Vboost / Vfloat          : 28.8 V / 27.6 V
#define NPB24_VOLT_MIN      21.0f
#define NPB24_VOLT_MAX      42.0f
#define NPB24_CURR_MIN       0.0f
#define NPB24_CURR_MAX      22.5f

// ── Command codes (datasheet CANBus command list, pages 9-10) ───────────────
enum NPBCmd : uint16_t {
    NPB_OPERATION       = 0x0000,  // R/W 1B  DC output ON/OFF
    NPB_VOUT_SET        = 0x0020,  // R/W 2B  direct output voltage setpoint (F=0.01).
                                    //  IGNORED while the charger is running its curve/
                                    //  battery-charging profile (CHG_STATUS shows CC/CV/FV
                                    //  stage) -- CURVE_CV governs voltage in that mode
                                    //  instead. Kept for completeness/direct-output-mode
                                    //  use; not used by this driver's normal charging path.
    NPB_IOUT_SET        = 0x0030,  // R/W 2B  direct output current setpoint (F=0.01).
                                    //  Same caveat as VOUT_SET above -- ignored during
                                    //  curve/battery charging; CURVE_CC governs current there.
    NPB_FAULT_STATUS    = 0x0040,  // R   2B  abnormal status bits
    NPB_READ_VOUT       = 0x0060,  // R   2B  measured output voltage  (F=0.01)
    NPB_READ_IOUT       = 0x0061,  // R   2B  measured output current  (F=0.01)
    NPB_READ_TEMP1      = 0x0062,  // R   2B  internal temperature     (F=0.1)
    NPB_CURVE_CC        = 0x00B0,  // R/W 2B  curve constant current   (F=0.01)
    NPB_CURVE_CV        = 0x00B1,  // R/W 2B  curve constant voltage   (F=0.01)
    NPB_CURVE_FV        = 0x00B2,  // R/W 2B  curve float voltage      (F=0.01)
    NPB_CURVE_TC        = 0x00B3,  // R/W 2B  curve taper current      (F=0.01)
    NPB_CURVE_CONFIG    = 0x00B4,  // R/W 2B  curve stage config
    NPB_CURVE_CC_TIMEOUT = 0x00B5, // R/W 2B  CC-stage timeout (see setter comment on units)
    NPB_CURVE_CV_TIMEOUT = 0x00B6, // R/W 2B  CV-stage timeout
    NPB_CURVE_FV_TIMEOUT = 0x00B7, // R/W 2B  FV-stage (float) timeout
    NPB_CHG_STATUS      = 0x00B8,  // R   2B  charging status report
    NPB_CHG_RST_VBAT    = 0x00B9,  // R/W 2B  voltage point to auto-restart charging (F=0.01)
    NPB_SYSTEM_STATUS   = 0x00C1,  // R   2B  system status
    NPB_SYSTEM_CONFIG   = 0x00C2,  // R/W 2B  system config (EEP_OFF, OPERATION_INIT, RSTE, etc.)
};

// ── Fault status bit masks (0x0040) ─────────────────────────────────────────
// Bit meanings per MEAN WELL FAULT_STATUS; verify against your firmware rev.
#define NPB_FAULT_OTP       (1 << 1)   // over-temperature
#define NPB_FAULT_OVP       (1 << 2)   // over-voltage
#define NPB_FAULT_OLP       (1 << 3)   // over-current / short
#define NPB_FAULT_SHORT     (1 << 4)   // output short
#define NPB_FAULT_AC        (1 << 5)   // AC abnormal
// These two are status bits, not fault conditions -- included in
// faultToString()'s output for completeness, but NOT counted by isFaulted().
#define NPB_STATUS_OP_OFF   (1 << 6)   // output currently turned off
#define NPB_STATUS_HI_TEMP  (1 << 7)   // internal temp warning (~95C); unit keeps running

// ── Charge status bit masks (0x00B8, CHG_STATUS) ────────────────────────────
// Low byte: what stage/mode the charger is currently in.
#define NPB_CHG_FULLM       (1 << 0)   // battery reports fully charged
#define NPB_CHG_CCM         (1 << 1)   // currently in constant-current stage
#define NPB_CHG_CVM         (1 << 2)   // currently in constant-voltage stage
#define NPB_CHG_FVM         (1 << 3)   // currently in float stage
#define NPB_CHG_WAKEUP_STOP (1 << 6)   // 0=wake-up finished, 1=still waking up
#define NPB_CHG_HI_TEMP_LO  (1 << 7)   // internal temp warning (low-byte copy)
// High byte (shifted left 8 to align with the 16-bit CHG_STATUS word):
#define NPB_CHG_NTCER       (1 << 10)  // NTC/temp-compensation wiring shorted
#define NPB_CHG_BTNC        (1 << 11)  // no battery detected
#define NPB_CHG_CCTOF       (1 << 13)  // CC-stage timeout occurred
#define NPB_CHG_CVTOF       (1 << 14)  // CV-stage timeout occurred
#define NPB_CHG_FVTOF       (1 << 15)  // float-stage timeout occurred

// ── Snapshot of everything read back from the charger ───────────────────────
struct ChargerData {
    bool     online;        // did we get any valid reply this poll cycle
    bool     outputOn;      // last commanded / reported OPERATION state
    float    vout;          // measured output voltage (V)
    float    iout;          // measured output current (A)
    float    temp;          // internal temperature (°C)
    uint16_t faultRaw;      // raw FAULT_STATUS word
    uint16_t chgStatus;     // raw CHG_STATUS word
    uint32_t lastRxMs;      // millis() of last good frame
};

class ChargerNPB {
public:
    ChargerNPB();

    // Bring up the TWAI driver on the given pins at 250 kbit/s.
    // chargerAddr defaults to 3 because floating A0/A1 pins read as logic 1
    // per the manual's addressing table, and logic 1/1 = Device 3. If you
    // later ground A0/A1 to GND(Signal) (pin 4) to force a different address,
    // pass the matching value here (0-3).
    // Returns false if the driver failed to install/start.
    bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint8_t chargerAddr = 0x03);

    // ── Control (writes) ─────────────────────────────────────────
    bool setOutput(bool on);            // OPERATION ON/OFF

    // Sends setOutput(wantOn), then polls FAULT_STATUS's OP_OFF bit until the
    // charger actually confirms it reached that state (or times out).
    // confirmedState is only valid when this returns true.
    //
    // NOTE: this is a BLOCKING call (up to timeoutMs, default 4s) -- it's
    // meant for one-off, user-initiated toggles (e.g. the serial console's
    // 'o' command) where a brief pause is fine. Do NOT call this from the
    // main loop()'s automatic/background logic (fault interlock, mismatch
    // correction, etc.) -- use plain setOutput() there so WiFi/OTA/WebSocket
    // servicing never gets stalled.
    bool setOutputConfirmed(bool wantOn, bool& confirmedState, uint32_t timeoutMs = 4000);

    // Forces freshly-written CURVE_CC/CURVE_CV/CURVE_FV/CURVE_TC (and the
    // other curve-family registers: CHG_RST_VBAT, the *_TIMEOUT registers)
    // to actually take effect NOW instead of waiting for the charger's next
    // AC power cycle. Per the manual, curve registers only latch in on a
    // remote/comm on-off toggle or AC re-power -- this does a confirmed
    // OFF then confirmed ON to force that toggle immediately.
    //
    // BLOCKING (up to ~2x timeoutMs) and briefly interrupts charging
    // (typically well under a second, up to a couple seconds if
    // FAULT_STATUS is slow to confirm). Only call this from user-initiated
    // one-off actions (console commands, WebUI setpoint changes) -- never
    // from loop()'s automatic/background logic.
    bool reapplyCurveNow(uint32_t timeoutMs = 4000);
    bool setVoltage(float volts);       // VOUT_SET   (clamped)
    bool setCurrent(float amps);        // IOUT_SET   (clamped)
    bool setCurveCV(float volts);       // CURVE_CV   (clamped)
    bool setCurveFV(float volts);       // CURVE_FV   (clamped)
    bool setCurveCC(float amps);        // CURVE_CC   (clamped)
    bool setCurveTC(float amps);        // CURVE_TC   (clamped)

    // NOTE: curve-related writes below (CURVE_CC/CV/FV/TC/*_TIMEOUT,
    // CHG_RST_VBAT) don't take effect immediately on the charger -- per the
    // manual they apply on the next AC power cycle, remote on/off toggle, or
    // communication on/off toggle. VOUT_SET/IOUT_SET/OPERATION, by contrast,
    // take effect immediately.

    // Voltage point at which the charger auto-restarts a charge cycle after
    // the battery has dropped from full. Requires SYSTEM_CONFIG's RSTE bit
    // to be enabled (see setSystemConfigRaw) to actually take effect.
    bool setChgRstVbat(float volts);            // CHG_RST_VBAT (clamped like voltage)

    // Per-stage timeout protection, in minutes. 0 disables the timeout for
    // that stage. The manual's own text is inconsistent about the valid
    // range (one section says 1-6000 minutes, a value-range table elsewhere
    // shows 60-64800) -- this is passed through as a raw register value
    // clamped to 0-6000 as the safer, better-documented interpretation.
    // Verify against your own charger's behavior before relying on it.
    bool setCurveCCTimeoutMinutes(uint16_t minutes);   // CURVE_CC_TIMEOUT
    bool setCurveCVTimeoutMinutes(uint16_t minutes);   // CURVE_CV_TIMEOUT
    bool setCurveFVTimeoutMinutes(uint16_t minutes);   // CURVE_FV_TIMEOUT

    // Advanced / raw access. CURVE_CONFIG and SYSTEM_CONFIG are multi-bit
    // registers (2/3 stage select, preset curve choice, EEP_OFF, RSTE,
    // OPERATION_INIT, etc.) -- rather than guess at every bit's exact
    // position from an imperfectly-OCR'd manual, these let you read the
    // current raw value and write back a modified one deliberately. See the
    // manual's CURVE_CONFIG/SYSTEM_CONFIG bit tables before using.
    // writeRaw/readRaw work for ANY command code, including the identification
    // registers (MFR_MODEL, MFR_SERIAL, etc.) this driver doesn't wrap.
    bool writeRaw(uint16_t cmd, uint16_t value);
    bool readRaw(uint16_t cmd, uint16_t& value);

    // ── Monitoring (reads) ───────────────────────────────────────
    // Each issues a request then waits briefly for the reply.
    bool readVoltage(float& out);
    bool readCurrent(float& out);
    bool readTemp(float& out);
    bool readFaultStatus(uint16_t& out);
    bool readChgStatus(uint16_t& out);
    bool readSystemStatus(uint16_t& out);   // SYSTEM_STATUS  (0x00C1)
    bool readCurveConfig(uint16_t& out);    // CURVE_CONFIG   (0x00B4)
    bool readSystemConfig(uint16_t& out);   // SYSTEM_CONFIG  (0x00C2)

    // Setpoint readback -- distinct from readVoltage()/readCurrent() above,
    // which read the charger's MEASURED output. These read back what the
    // charger has actually stored as VOUT_SET/IOUT_SET (both are R/W per the
    // manual), so you can confirm a setVoltage()/setCurrent() write really
    // took, rather than trusting the ESP32's own memory of what it sent.
    bool readVoltageSetpoint(float& out);   // VOUT_SET readback (0x0020)
    bool readCurrentSetpoint(float& out);   // IOUT_SET readback (0x0030)

    // Convenience: refresh the whole ChargerData snapshot in one call.
    void poll();
    const ChargerData& data() const { return _d; }

    // Diagnostic: dump TWAI bus/error status to the log. Distinguishes
    // "nothing on the bus is ACKing us" (TX error count climbing, bus errors)
    // from "something is ACKing but never replies with data we recognize"
    // (clean status, just no matching reply frames). Call this a few times
    // after a poll() cycle that logged "no reply".
    void logBusStatus();

    // Diagnostic: put the TWAI peripheral into LISTEN_ONLY mode (transmits
    // nothing, generates no ACKs, purely observes) for durationMs and logs
    // the raw ID + data bytes of anything seen on the bus. Useful when our
    // assumed frame format isn't getting a reply -- if the charger ever
    // broadcasts anything unprompted (a heartbeat/status frame), this shows
    // its REAL arbitration ID and payload layout instead of us guessing.
    // Call this BEFORE charger.begin() -- it installs/uninstalls its own
    // temporary driver instance and leaves the bus free afterward for the
    // normal begin() call.
    static void sniffBus(gpio_num_t txPin, gpio_num_t rxPin, uint32_t durationMs = 8000);

    // Decode helpers
    static bool isFaulted(uint16_t faultWord);
    static String faultToString(uint16_t faultWord);
    static String chgStatusToString(uint16_t chgStatusWord);

private:
    uint32_t _txId;         // pre-computed host->charger arbitration ID
    uint32_t _rxId;         // charger->host arbitration ID we listen for
    bool     _ready;
    ChargerData _d;

    // Low-level frame send. dataLen = payload length (2 for read, 4 for write).
    bool sendFrame(const uint8_t* payload, uint8_t len);

    // Send a read request for `cmd`, wait up to timeoutMs for the matching
    // reply, and return the 16-bit little-endian data word in `value`.
    bool requestValue(uint16_t cmd, uint16_t& value, uint32_t timeoutMs = 30);

    // Send a 16-bit write for `cmd`.
    bool writeValue(uint16_t cmd, uint16_t value);

    static float clampf(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};

extern ChargerNPB charger;