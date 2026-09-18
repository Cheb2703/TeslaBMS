#include "ChargerNPB.h"
#include "Logger.h"
#include "driver/twai.h"

// Global instance
ChargerNPB charger;

ChargerNPB::ChargerNPB() : _txId(0), _rxId(0), _ready(false) {
    memset(&_d, 0, sizeof(_d));
}

// ─────────────────────────────────────────────────────────────────────────────
//  begin — install & start the TWAI driver
// ─────────────────────────────────────────────────────────────────────────────
bool ChargerNPB::begin(gpio_num_t txPin, gpio_num_t rxPin, uint8_t chargerAddr) {
    // Per MEAN WELL NPB/NPP User Manual Chapter 6.1/6.2:
    //   Controller -> Charger (our TX): 0x000C0100 | chargerAddr
    //   Charger -> Controller (our RX): 0x000C0000 | chargerAddr
    // There is no separate "host address" in this protocol -- only a fixed
    // direction byte (0x01 outbound / 0x00 inbound) plus the charger's own
    // 2-bit address (0-3, from A0/A1).
    _txId = 0x000C0100u | (chargerAddr & 0x03);
    _rxId = 0x000C0000u | (chargerAddr & 0x03);

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin,
                                                          TWAI_MODE_NORMAL);
    // 250 kbit/s is the MEAN WELL fixed bus speed.
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    // Accept everything; we filter by ID in software (only a few IDs on-bus).
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Logger::error("TWAI driver install failed");
        _ready = false;
        return false;
    }
    if (twai_start() != ESP_OK) {
        Logger::error("TWAI start failed");
        twai_driver_uninstall();
        _ready = false;
        return false;
    }

    Logger::info("Charger CAN up. TX id=%X RX id=%X", _txId, _rxId);
    _ready = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Low-level frame TX
// ─────────────────────────────────────────────────────────────────────────────
bool ChargerNPB::sendFrame(const uint8_t* payload, uint8_t len) {
    if (!_ready || len > 8) return false;

    twai_message_t msg = {};
    msg.identifier       = _txId;
    msg.extd             = 1;        // 29-bit extended frame (CAN 2.0B)
    msg.rtr              = 0;
    msg.data_length_code = len;
    memcpy(msg.data, payload, len);

    // Block up to 20 ms for a TX mailbox slot.
    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(20));
    if (err != ESP_OK) {
        // This distinguishes "frame never left the ESP32" (queue full, driver
        // not running, invalid state) from "frame went out but nobody ACKed
        // it" -- the latter shows up as tx_error_counter/bus_error_count
        // climbing in logBusStatus() instead. A loose/intermittent physical
        // connection can look like the driver being fine but individual
        // transmits still failing to queue.
        Logger::warn("Charger CAN: twai_transmit() failed, err=%d", (int)err);
        return false;
    }
    return true;
}

// Write a 16-bit value:  [cmd_lo, cmd_hi, data_lo, data_hi]
bool ChargerNPB::writeValue(uint16_t cmd, uint16_t value) {
    uint8_t p[4] = {
        (uint8_t)(cmd   & 0xFF), (uint8_t)(cmd   >> 8),
        (uint8_t)(value & 0xFF), (uint8_t)(value >> 8),
    };
    return sendFrame(p, 4);
}

// Read: send [cmd_lo, cmd_hi], then wait for a reply with our RX id whose
// first two payload bytes echo the command code. Returns the 16-bit LE word.
bool ChargerNPB::requestValue(uint16_t cmd, uint16_t& value, uint32_t timeoutMs) {
    if (!_ready) return false;

    uint8_t req[2] = { (uint8_t)(cmd & 0xFF), (uint8_t)(cmd >> 8) };
    if (!sendFrame(req, 2)) return false;

    uint32_t deadline = millis() + timeoutMs;
    twai_message_t rx;
    while ((int32_t)(millis() - deadline) < 0) {
        if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK) {
            if (rx.identifier == _rxId && rx.data_length_code >= 4) {
                uint16_t echo = rx.data[0] | (rx.data[1] << 8);
                if (echo == cmd) {
                    value = rx.data[2] | (rx.data[3] << 8);
                    _d.lastRxMs = millis();
                    _d.online   = true;
                    return true;
                }
            }
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Control (writes)
// ─────────────────────────────────────────────────────────────────────────────
bool ChargerNPB::setOutput(bool on) {
    // OPERATION is a 1-byte command; send [cmd_lo, cmd_hi, state].
    uint8_t p[3] = { (uint8_t)(NPB_OPERATION & 0xFF),
                     (uint8_t)(NPB_OPERATION >> 8),
                     (uint8_t)(on ? 0x01 : 0x00) };
    bool ok = sendFrame(p, 3);
    if (ok) _d.outputOn = on;
    return ok;
}

// Blocking helper: command the output, then actually poll FAULT_STATUS's
// OP_OFF bit until the charger confirms it got there, instead of just
// trusting that the CAN write went out. See header comment for why this is
// only meant for one-off console-triggered toggles, not loop() automation.
bool ChargerNPB::setOutputConfirmed(bool wantOn, bool& confirmedState, uint32_t timeoutMs) {
    if (!setOutput(wantOn)) return false;

    uint32_t deadline = millis() + timeoutMs;
    while ((int32_t)(millis() - deadline) < 0) {
        delay(200);
        uint16_t fault;
        if (readFaultStatus(fault)) {
            bool actuallyOff = (fault & NPB_STATUS_OP_OFF) != 0;
            bool actuallyOn  = !actuallyOff;
            if (actuallyOn == wantOn) {
                confirmedState = wantOn;
                _d.outputOn = wantOn;
                return true;
            }
        }
    }
    return false; // caller should not trust/update its cached state
}

// Forces curve-family register writes (CURVE_CC/CV/FV/TC, CHG_RST_VBAT, the
// *_TIMEOUT registers) to take effect right now via a confirmed off-then-on
// toggle, instead of waiting for the charger's next AC power cycle. See
// header comment for why this is blocking and console/WebUI-only.
bool ChargerNPB::reapplyCurveNow(uint32_t timeoutMs) {
    bool confirmed;
    if (!setOutputConfirmed(false, confirmed, timeoutMs)) return false;
    delay(100); // brief settle before re-enabling
    return setOutputConfirmed(true, confirmed, timeoutMs);
}

bool ChargerNPB::setVoltage(float volts) {
    volts = clampf(volts, NPB24_VOLT_MIN, NPB24_VOLT_MAX);
    return writeValue(NPB_VOUT_SET, (uint16_t)(volts * 100.0f + 0.5f));
}

bool ChargerNPB::setCurrent(float amps) {
    amps = clampf(amps, NPB24_CURR_MIN, NPB24_CURR_MAX);
    return writeValue(NPB_IOUT_SET, (uint16_t)(amps * 100.0f + 0.5f));
}

bool ChargerNPB::setCurveCV(float volts) {
    volts = clampf(volts, NPB24_VOLT_MIN, NPB24_VOLT_MAX);
    return writeValue(NPB_CURVE_CV, (uint16_t)(volts * 100.0f + 0.5f));
}

bool ChargerNPB::setCurveFV(float volts) {
    volts = clampf(volts, NPB24_VOLT_MIN, NPB24_VOLT_MAX);
    return writeValue(NPB_CURVE_FV, (uint16_t)(volts * 100.0f + 0.5f));
}

bool ChargerNPB::setCurveCC(float amps) {
    amps = clampf(amps, NPB24_CURR_MIN, NPB24_CURR_MAX);
    return writeValue(NPB_CURVE_CC, (uint16_t)(amps * 100.0f + 0.5f));
}

bool ChargerNPB::setCurveTC(float amps) {
    amps = clampf(amps, NPB24_CURR_MIN, NPB24_CURR_MAX);
    return writeValue(NPB_CURVE_TC, (uint16_t)(amps * 100.0f + 0.5f));
}

bool ChargerNPB::setChgRstVbat(float volts) {
    volts = clampf(volts, NPB24_VOLT_MIN, NPB24_VOLT_MAX);
    return writeValue(NPB_CHG_RST_VBAT, (uint16_t)(volts * 100.0f + 0.5f));
}

bool ChargerNPB::setCurveCCTimeoutMinutes(uint16_t minutes) {
    if (minutes > 6000) minutes = 6000;
    return writeValue(NPB_CURVE_CC_TIMEOUT, minutes);
}

bool ChargerNPB::setCurveCVTimeoutMinutes(uint16_t minutes) {
    if (minutes > 6000) minutes = 6000;
    return writeValue(NPB_CURVE_CV_TIMEOUT, minutes);
}

bool ChargerNPB::setCurveFVTimeoutMinutes(uint16_t minutes) {
    if (minutes > 6000) minutes = 6000;
    return writeValue(NPB_CURVE_FV_TIMEOUT, minutes);
}

bool ChargerNPB::writeRaw(uint16_t cmd, uint16_t value) {
    return writeValue(cmd, value);
}

bool ChargerNPB::readRaw(uint16_t cmd, uint16_t& value) {
    return requestValue(cmd, value);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Monitoring (reads)
// ─────────────────────────────────────────────────────────────────────────────
bool ChargerNPB::readVoltage(float& out) {
    uint16_t raw;
    if (!requestValue(NPB_READ_VOUT, raw)) return false;
    out = raw * 0.01f;          // F = 0.01
    _d.vout = out;
    return true;
}

bool ChargerNPB::readCurrent(float& out) {
    uint16_t raw;
    if (!requestValue(NPB_READ_IOUT, raw)) return false;
    out = raw * 0.01f;          // F = 0.01
    _d.iout = out;
    return true;
}

bool ChargerNPB::readTemp(float& out) {
    uint16_t raw;
    if (!requestValue(NPB_READ_TEMP1, raw)) return false;
    // Temperature is signed, F = 0.1.
    out = (int16_t)raw * 0.1f;
    _d.temp = out;
    return true;
}

bool ChargerNPB::readFaultStatus(uint16_t& out) {
    if (!requestValue(NPB_FAULT_STATUS, out)) return false;
    _d.faultRaw = out;
    // Ground truth for outputOn comes from the charger's own OP_OFF bit here,
    // not from our own command history. setOutput()/setOutputConfirmed() set
    // outputOn optimistically the instant we send a command, but THIS is what
    // gets overwritten on every poll() cycle with what the charger actually
    // reports -- which is what lets enforceChargerDesiredState() in main.cpp
    // detect a real mismatch (e.g. the charger auto-restarting its own output
    // after an AC power cycle) instead of comparing our intent against an
    // echo of itself.
    _d.outputOn = (out & NPB_STATUS_OP_OFF) == 0;
    return true;
}

bool ChargerNPB::readChgStatus(uint16_t& out) {
    if (!requestValue(NPB_CHG_STATUS, out)) return false;
    _d.chgStatus = out;
    return true;
}

bool ChargerNPB::readSystemStatus(uint16_t& out) {
    return requestValue(NPB_SYSTEM_STATUS, out);
}

bool ChargerNPB::readCurveConfig(uint16_t& out) {
    return requestValue(NPB_CURVE_CONFIG, out);
}

bool ChargerNPB::readSystemConfig(uint16_t& out) {
    return requestValue(NPB_SYSTEM_CONFIG, out);
}

bool ChargerNPB::readVoltageSetpoint(float& out) {
    uint16_t raw;
    if (!requestValue(NPB_VOUT_SET, raw)) return false;
    out = raw * 0.01f;
    return true;
}

bool ChargerNPB::readCurrentSetpoint(float& out) {
    uint16_t raw;
    if (!requestValue(NPB_IOUT_SET, raw)) return false;
    out = raw * 0.01f;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  poll — refresh the whole snapshot. Marks offline if nothing replies.
// ─────────────────────────────────────────────────────────────────────────────
void ChargerNPB::poll() {
    if (!_ready) { _d.online = false; return; }

    bool any = false;
    float f;  uint16_t w;

    if (readVoltage(f))     any = true;
    if (readCurrent(f))     any = true;
    if (readTemp(f))        any = true;
    if (readFaultStatus(w)) any = true;
    if (readChgStatus(w))   any = true;

    _d.online = any;
    if (!any) {
        Logger::warn("Charger CAN: no reply this cycle");
        logBusStatus();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  logBusStatus — dump TWAI-level health, independent of whether the charger
//  ever sends back data we recognize. This tells us whether anything on the
//  bus is ACKing our frames at the CAN protocol level at all.
// ─────────────────────────────────────────────────────────────────────────────
void ChargerNPB::logBusStatus() {
    if (!_ready) {
        Logger::warn("Charger CAN: driver not started, nothing to report");
        return;
    }

    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        Logger::error("Charger CAN: twai_get_status_info() failed");
        return;
    }

    const char* stateStr = "UNKNOWN";
    switch (st.state) {
        case TWAI_STATE_STOPPED:    stateStr = "STOPPED";    break;
        case TWAI_STATE_RUNNING:    stateStr = "RUNNING";    break;
        case TWAI_STATE_BUS_OFF:    stateStr = "BUS_OFF";    break;
        case TWAI_STATE_RECOVERING: stateStr = "RECOVERING"; break;
    }

    Logger::info("Charger CAN status: state=%s  txErr=%d  rxErr=%d  txQueued=%d  rxQueued=%d  txFailed=%d  arbLost=%d  busErr=%d",
                 stateStr,
                 (int)st.tx_error_counter, (int)st.rx_error_counter,
                 (int)st.msgs_to_tx, (int)st.msgs_to_rx,
                 (int)st.tx_failed_count, (int)st.arb_lost_count,
                 (int)st.bus_error_count);

    // Interpretation guide, logged once for convenience:
    //   tx_error_counter climbing toward 128+ / state BUS_OFF
    //     -> nothing on the bus is ACKing frames at all (wiring, termination,
    //        transceiver pairing, or the charger's CAN interface isn't live).
    //   tx_error_counter low/zero, state RUNNING, but poll() still gets no
    //   data reply
    //     -> something IS acknowledging our frames at the bus level, but our
    //        assumed frame format (arbitration ID / address / command layout)
    //        doesn't match what the charger expects. That points at the
    //        protocol assumptions, not the wiring.
    if (st.state == TWAI_STATE_BUS_OFF) {
        Logger::warn("Charger CAN: BUS_OFF -- no device is acknowledging frames. Check wiring/termination/transceiver pairing.");
    } else if (st.tx_error_counter > 0) {
        Logger::warn("Charger CAN: tx_error_counter=%d -- frames are going unacknowledged.", (int)st.tx_error_counter);
    } else {
        Logger::info("Charger CAN: bus healthy (errors=0) -- if poll() still gets no reply, the issue is likely frame format/address, not wiring.");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  sniffBus — listen-only capture, run BEFORE begin(). Installs its own
//  temporary TWAI driver in LISTEN_ONLY mode (transmits nothing, generates no
//  ACKs), logs the raw ID/data of anything it sees for durationMs, then tears
//  itself down so the normal begin() call gets a clean bus afterward.
// ─────────────────────────────────────────────────────────────────────────────
void ChargerNPB::sniffBus(gpio_num_t txPin, gpio_num_t rxPin, uint32_t durationMs) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin,
                                                          TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Logger::error("Charger CAN sniff: driver install failed");
        return;
    }
    if (twai_start() != ESP_OK) {
        Logger::error("Charger CAN sniff: start failed");
        twai_driver_uninstall();
        return;
    }

    Logger::info("Charger CAN sniff: listening for %lu ms (no frames will be transmitted)...",
                 (unsigned long)durationMs);

    uint32_t deadline = millis() + durationMs;
    int framesSeen = 0;
    twai_message_t rx;

    while ((int32_t)(millis() - deadline) < 0) {
        if (twai_receive(&rx, pdMS_TO_TICKS(50)) == ESP_OK) {
            framesSeen++;
            // Print raw ID (hex) + DLC + every data byte (hex) -- this is the
            // ground-truth format if the charger sends anything on its own.
            char dataStr[32] = "";
            char byteBuf[6];
            for (int i = 0; i < rx.data_length_code && i < 8; i++) {
                snprintf(byteBuf, sizeof(byteBuf), "%02X ", rx.data[i]);
                strncat(dataStr, byteBuf, sizeof(dataStr) - strlen(dataStr) - 1);
            }
            Logger::info("Charger CAN sniff: id=%X ext=%d dlc=%d data=%s",
                         (unsigned int)rx.identifier, rx.extd, rx.data_length_code, dataStr);
        }
    }

    if (framesSeen == 0) {
        Logger::warn("Charger CAN sniff: saw NOTHING in %lu ms. The charger is not "
                     "transmitting anything unprompted -- it likely only replies to "
                     "requests, so this doesn't tell us the ID format directly.",
                     (unsigned long)durationMs);
    } else {
        Logger::info("Charger CAN sniff: captured %d frame(s) total.", framesSeen);
    }

    twai_stop();
    twai_driver_uninstall();
}

bool ChargerNPB::isFaulted(uint16_t f) {
    return (f & (NPB_FAULT_OTP | NPB_FAULT_OVP | NPB_FAULT_OLP |
                 NPB_FAULT_SHORT | NPB_FAULT_AC)) != 0;
}

String ChargerNPB::faultToString(uint16_t f) {
    String s;
    // Actual fault conditions first.
    if (f & NPB_FAULT_OTP)   s += "OTP ";
    if (f & NPB_FAULT_OVP)   s += "OVP ";
    if (f & NPB_FAULT_OLP)   s += "OLP ";
    if (f & NPB_FAULT_SHORT) s += "SHORT ";
    if (f & NPB_FAULT_AC)    s += "AC ";
    // Status bits -- not faults, but worth showing so "0x0040" isn't blank.
    if (f & NPB_STATUS_OP_OFF)  s += "OUTPUT_OFF ";
    if (f & NPB_STATUS_HI_TEMP) s += "HIGH_TEMP_WARN ";
    if (s.length() == 0) s = "OK";
    s.trim();
    return s;
}

// Decode CHG_STATUS (0x00B8) into a human-readable summary of what stage the
// charger is currently in, plus any timeout/detection flags. See the bit
// masks above (NPB_CHG_*) -- high byte bits are pre-shifted left 8 so they
// line up with the raw 16-bit register value as read off the wire.
String ChargerNPB::chgStatusToString(uint16_t c) {
    String s;
    if (c & NPB_CHG_BTNC)        s += "NO_BATTERY ";
    if (c & NPB_CHG_CCM)         s += "CC_STAGE ";
    if (c & NPB_CHG_CVM)         s += "CV_STAGE ";
    if (c & NPB_CHG_FVM)         s += "FLOAT_STAGE ";
    if (c & NPB_CHG_FULLM)       s += "FULLY_CHARGED ";
    if (c & NPB_CHG_WAKEUP_STOP) s += "WAKING_UP ";
    if (c & NPB_CHG_NTCER)       s += "NTC_WIRING_SHORTED ";
    if (c & NPB_CHG_CCTOF)       s += "CC_TIMEOUT ";
    if (c & NPB_CHG_CVTOF)       s += "CV_TIMEOUT ";
    if (c & NPB_CHG_FVTOF)       s += "FLOAT_TIMEOUT ";
    if (c & NPB_CHG_HI_TEMP_LO)  s += "HIGH_TEMP_WARN ";
    if (s.length() == 0) s = "IDLE";
    s.trim();
    return s;
}