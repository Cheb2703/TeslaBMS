#pragma once

#include <Arduino.h>
#include "Webui.h"

// -- Access control switches ---------------------------------------------------
// 1 = require it, 0 = don't. Both are OFF so you can join the AP and open the
// web UI without entering anything. Flip back to 1 to restore the old behavior
// (the saved passwords in secrets.h / flash are left untouched, so they come
// straight back). Anyone in Wi-Fi range can reach the device when these are 0.
#define AP_REQUIRE_PASSWORD  0   // 0 = open Wi-Fi access point (no password)
#define WEBUI_REQUIRE_AUTH   0   // 0 = web UI without username/password prompt

// -- BMS supervision -----------------------------------------------------------
// How many Tesla modules the pack has. This is only the first-boot default;
// change it in the web UI Settings tab afterwards. If fewer modules than this
// are found, a fault is raised and the charger is held off.
#define DEFAULT_PACKS_CONFIGURED  2

// Number of cells in series in the pack (a Tesla module is 6S; two modules in
// PARALLEL are still 6S). The charger's voltage settings are capped at
// CELLS_IN_SERIES x the cell over-voltage limit, so a bad setting or a typo
// can never ask a 6S pack for the charger's full 42 V.
#define CELLS_IN_SERIES           6

// Highest value the cell over-voltage limit (VOLTLIMHI) may be set to. Tesla
// NMC cells are 4.20 V max; the console used to accept up to 6.0 V, which
// would silently switch over-voltage protection off.
#define CELL_VOLT_ABS_MAX         4.25f

// Lowest cell voltage at which the charger is still allowed to run. A cell
// below the under-voltage limit (VOLTLIMLO, 3.30 V by default) sounds the alarm
// but must NOT stop charging, or a low pack could never be recharged. Below
// this floor a cell is considered deeply discharged and charging is blocked.
#define CELL_CHARGE_MIN_V         2.5f

// If loop() stops running for this long (a hang, an endless loop), the ESP32
// reboots itself. The charger output is switched off during boot. Must be
// longer than the slowest legitimate blocking call (a charger curve re-apply
// can take about 8 s, and a poll with several unresponsive modules a few more).
#define LOOP_WDT_TIMEOUT_S        30

// Each time the charger output switches on, its curve settings (CC/CV/FV/TC)
// are read back and compared with what was sent. If they don't match:
//   0 = only write an error to the log (default while this is being proven)
//   1 = also switch the charger off and leave it off
#define CHARGER_VERIFY_TURNS_OFF  0

// A module that fails this many 3-second read cycles in a row (about 10 s)
// raises a communication fault, so the charger is not left running on stale
// readings if a BMB cable comes loose or a board stops answering.
#define BMB_COMM_FAIL_LIMIT       3

//extern HardwareSerial Serial1; // Leftover from back in the day when this ran on arduino

//Set to the proper port for your USB connection - SerialUSB on Due (Native) or Serial for Due (Programming) or Teensy
// Routed through WebSerialTee (defined in WebUI.h) so everything printed
// here (Logger's debug/info/warn/error/console output, the menu, input
// echo) also reaches the web UI's Console tab -- USB behavior is unchanged,
// this only adds a second destination. See WebUI.h for the exact scope of
// what is/isn't captured, and a note on the extra include weight this adds
// to every file that includes config.h.
#define SERIALCONSOLE   webSerialTee

//Define this to be the serial port the Tesla BMS modules are connected to.
//On the Due you need to use a USART port (Serial1, Serial2, Serial3) and update the call to serialSpecialInit if not Serial1
#define SERIAL  Serial1

#define REG_DEV_STATUS      0
#define REG_GPAI            1
#define REG_VCELL1          3
#define REG_VCELL2          5
#define REG_VCELL3          7
#define REG_VCELL4          9
#define REG_VCELL5          0xB
#define REG_VCELL6          0xD
#define REG_TEMPERATURE1    0xF
#define REG_TEMPERATURE2    0x11
#define REG_ALERT_STATUS    0x20
#define REG_FAULT_STATUS    0x21
#define REG_COV_FAULT       0x22
#define REG_CUV_FAULT       0x23
#define REG_ADC_CTRL        0x30
#define REG_IO_CTRL         0x31
#define REG_BAL_CTRL        0x32
#define REG_BAL_TIME        0x33
#define REG_ADC_CONV        0x34
#define REG_ADDR_CTRL       0x3B

#define MAX_MODULE_ADDR     0x3E //0x3E

#define EEPROM_VERSION      0x10    //update any time EEPROM struct below is changed.



typedef struct {
    uint8_t version;
    uint8_t checksum;
    uint8_t logLevel;
    float OverVSetpoint;
    float UnderVSetpoint;
    float OverTSetpoint;
    float UnderTSetpoint;
    float balanceVoltage;
    float balanceHyst;
} EEPROMSettings;

