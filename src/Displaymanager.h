#pragma once

#include <Arduino.h>
#include <LovyanGFX.hpp>

// ── Panel subclass: safe command-only reinit ────────────────────────────────
// LGFX_Device::init() -> Panel_LCD::init() -> Panel_Device::init() ALWAYS
// calls _light->init(0), which calls Arduino-ESP32's ledcAttach() on the
// backlight pin -- confirmed by reading the actual LovyanGFX v1 and
// arduino-esp32 core source. ledcAttach() is not safe to call a second time
// on a pin that's already attached (the core logs "Pin X is already
// attached to LEDC" and does not cleanly reconfigure it), and this is NOT
// guarded anywhere in the call chain. This is what broke the display on
// real hardware when the periodic watchdog called _lcd.init() a second
// time -- confirmed against a matching report in LovyanGFX's own GitHub
// discussions ("re-init displays" #370): a maintainer's own words were
// "multiple tft.init() is a bit messy."
//
// getInitCommands()/command_list() (used internally by Panel_LCD::init() to
// actually push the ST7796S register sequence) are `protected` in the
// library -- not reachable from outside the panel class. This subclass
// exposes a public wrapper that calls only those two, deliberately skipping
// Panel_Device::init() (and therefore skipping the backlight re-attach and
// the RST/CS pinMode setup, both of which are already done once at boot and
// don't need repeating). The caller is responsible for the RST hardware
// pulse itself (see DisplayManager::reinitPanel()) -- this class only
// resends the command list afterward.
class Panel_ST7796_CmdReinit : public lgfx::Panel_ST7796 {
public:
    void resendInitCommands() {
        startWrite(true);
        for (uint8_t i = 0; auto cmds = getInitCommands(i); i++) {
            command_list(cmds);
        }
        endWrite();
    }
};

// ── Pin config for ESP32-S3 + ST7796S 4.0" 480×320 ──────────────────────────
class LGFX : public lgfx::LGFX_Device {
    Panel_ST7796_CmdReinit _panel;
    lgfx::Bus_SPI       _bus;
    lgfx::Light_PWM     _light;

public:
    // Command-list-only reinit -- see Panel_ST7796_CmdReinit above for why
    // this exists instead of calling init() again.
    void resendInitCommandsOnly() { _panel.resendInitCommands(); }

    LGFX() {
        {
            auto cfg        = _bus.config();
            cfg.spi_host    = SPI2_HOST;
            cfg.spi_mode    = 0;
            cfg.freq_write  = 40000000;
            cfg.freq_read   = 16000000;
            cfg.spi_3wire   = false;
            cfg.use_lock    = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk    = 11;
            cfg.pin_mosi    = 12;
            cfg.pin_miso    = -1;
            cfg.pin_dc      = 13;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg        = _panel.config();
            cfg.pin_cs      = 14;
            cfg.pin_rst     = 9;
            cfg.pin_busy    = -1;
            cfg.memory_width  = 320;
            cfg.memory_height = 480;
            cfg.panel_width   = 320;
            cfg.panel_height  = 480;
            cfg.offset_x      = 0;
            cfg.offset_y      = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits  = 1;
            cfg.readable      = false;
            cfg.invert        = false;
            cfg.rgb_order     = false;
            cfg.dlen_16bit    = false;
            cfg.bus_shared    = false;
            _panel.config(cfg);
        }
        {
            auto cfg        = _light.config();
            cfg.pin_bl      = 10;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light.config(cfg);
            _panel.setLight(&_light);
        }
        setPanel(&_panel);
    }
};

// ── Colour palette (RGB565) ──────────────────────────────────────────────────
#define COL_BG          0x0841  // #0d0d0d  near-black
#define COL_SURFACE     0x18C3  // #141414  module panels
#define COL_BORDER      0x39E7  // #3a3a3a  hairlines (brighter than before)
#define COL_GREEN       0x3E68  // #3cf08f  brighter, saturated green — was too dim
#define COL_GREEN_DIM   0x0D43  // #0d2b1a  SoC box bg
#define COL_AMBER       0xFD00  // #ffa500  punchier orange, also used for balancing now
#define COL_BLUE        0x5CF3  // #5b9cf6  cell low
#define COL_RED         0xF8A2  // #ff5454  brighter red, was too pink/dull
#define COL_MUTED       0x9CD3  // #9a9a9a  labels — was nearly invisible (0x52AA), now actually legible
#define COL_DIM         0x6B4D  // #6a6a6a
#define COL_WHITE       0xFFFF

// ── Screen orientation ──────────────────────────────────────────────────────
// LovyanGFX rotation, 0-3 (each step turns the picture 90 degrees). 1 and 3 are
// both landscape and are 180 degrees apart: swap between them if the picture
// is upside down for how the display is mounted. Used at start-up AND by the
// periodic panel re-init, so it must only ever be set here.
#define LCD_ROTATION 3

// ── Display dimensions ───────────────────────────────────────────────────────
#define DISP_W  480
#define DISP_H  320

// ── Layout constants ─────────────────────────────────────────────────────────
#define SOC_BOX_X    8
#define SOC_BOX_Y    8
#define SOC_BOX_W   100
#define SOC_BOX_H    54

#define TOPBAR_Y     9
#define TOPBAR_H     54
#define STATS_X     116
#define STATS_Y       8

#define MOD_Y        73   // top of module panels
#define MOD_H       212   // height of module panels
#define MOD1_X        8
#define MOD2_X      244
#define MOD_W       228

#define STATUS_Y    288

// How long each page stays up before the display moves to the next one. The
// dashboard is always in the rotation; the fault page joins it while any fault
// is active, and the charging page while the charger is on. With more than one
// page in the rotation they take turns (dashboard -> faults -> charging ->
// dashboard ...), so nothing hogs the screen and live pack data stays visible.
#define PAGE_ROTATE_INTERVAL_MS 8000

// Periodic proactive panel reinit ("display watchdog"). This board has no
// MISO/readback wired to the LCD (write-only SPI bus), so there's no way to
// ask the ST7796S "are you actually alive" and get a real answer -- SPI
// writes complete successfully even if the panel has locked up or is
// garbling data. Instead of true freeze detection, we manually pulse RST
// (GPIO9) and resend just the ST7796S init command list on a fixed
// interval, as cheap insurance against SPI/EMI-induced corruption in a
// vibration-heavy field environment. See Panel_ST7796_CmdReinit above for
// why this does NOT call the library's top-level _lcd.init() (confirmed
// unsafe to call twice -- backlight LEDC re-attach). This is blind by
// design -- it resets on a timer, not a confirmed fault -- so keep the
// interval long enough that a mid-reinit blank moment is never mistaken
// for a real fault by anyone glancing at the unit.
#define DISPLAY_REINIT_INTERVAL_MS (15UL * 60UL * 1000UL)  // 15 minutes

// Maximum number of simultaneous faults the fault page can show at once.
// If more than this are active, the page shows a "+N more" note and points
// to the serial console for the full list.
#define MAX_DISPLAY_FAULTS 6

// Rough pack capacity used ONLY for the charging page's "estimated time to
// full" figure. This is NOT measured -- it's an assumption you should adjust
// to match your actual modules' real capacity for a meaningful estimate.
// The ETA math is intentionally simple (linear extrapolation from SoC gap
// and present current draw) and will be optimistic during the CV/float
// taper, where current drops well before the pack is actually full.
#define PACK_CAPACITY_AH 238.0f

// ── Data struct the display consumes ────────────────────────────────────────
struct DisplayData {
    // Pack-level
    float   packVoltage;
    float   cellLow;
    float   cellHigh;
    float   avgTemp;
    uint8_t socPercent;       // 0-100, OCV-estimated
    uint32_t uptimeSeconds;
    int     numModules;
    int     goodPackets;
    int     badPackets;
    int     balancingCount;
    bool    isFaulted;
    bool    chargerBlocked;   // true if any active fault holds the charger off (isFaulted also counts alarm-only faults)

    // Active faults -- the fault PAGE (drawn instead of the normal dashboard
    // while isFaulted is true) lists each of these with its own duration.
    int      activeFaultCount;                      // total active (may exceed MAX_DISPLAY_FAULTS)
    char     faultReasons[MAX_DISPLAY_FAULTS][40];
    uint32_t faultDurationsMs[MAX_DISPLAY_FAULTS];

    // Fault history -- tracks the most recently *cleared* fault event so a
    // momentary blip is still visible even though the live status is
    // non-latching. hasFaultHistory is false until the first fault clears.
    bool     hasFaultHistory;
    uint32_t lastFaultDurationMs;      // how long the most recent fault lasted
    uint32_t secondsSinceFaultCleared; // how long ago it cleared
    char     lastFaultReason[40];      // what it actually was, snapshotted right before it cleared
    int      lastFaultCount;           // how many faults were active at that moment (may be >1)

    // Per-module (2 modules, 6 cells each)
    float   cellVolt[2][6];
    bool    cellBalancing[2][6];
    float   moduleVolt[2];
    float   tempNeg[2];
    float   tempPos[2];

    // Charger status -- populated in main.cpp from ChargerNPB::data() every
    // 1-second display-build cycle. chargerPresent is false for the whole
    // run if charger.begin() never succeeded at boot (no CAN transceiver /
    // wiring fault), in which case the charging page never shows regardless
    // of the other fields below.
    bool     chargerPresent;      // charger.begin() succeeded at boot
    bool     chargerOnline;       // got a valid CAN reply this poll cycle
    bool     chargerOutputOn;     // last known OPERATION (DC output) state
    float    chargerVout;         // measured output voltage (V)
    float    chargerIout;         // measured output current (A)
    float    chargerTemp;         // charger internal temperature (C)
    float    chargerVoltSetpoint; // commanded VOUT_SET
    float    chargerCurrSetpoint; // commanded IOUT_SET
    bool     chargerFaulted;      // true FAULT_STATUS bits only (not the status-only bits)
    char     chargerFaultStr[40]; // human-readable FAULT_STATUS, "OK" if none
    uint16_t chargerChgStatusRaw; // raw CHG_STATUS word -- decoded in DisplayManager
    uint32_t chargerLastRxAgoMs;  // ms since last good CAN frame (0 if never online)
};

// ── Manager class ─────────────────────────────────────────────────────────────
class DisplayManager {
public:
    DisplayManager();
    void begin();
    void update(const DisplayData& d);

private:
    LGFX    _lcd;
    bool    _ready;

    // Which page is currently on screen, and when we switched to it. A new
    // fault (or the start of charging) is shown immediately; after that the
    // pages in the rotation take turns every PAGE_ROTATE_INTERVAL_MS. Also used
    // to detect transitions back to PAGE_DASHBOARD so we know when to clear
    // stale full-screen content out of the gaps the dashboard's partial-redraw
    // sprites don't cover.
    enum DisplayPage { PAGE_DASHBOARD, PAGE_FAULT, PAGE_CHARGING };
    DisplayPage _currentPage;
    uint32_t    _pageEnteredMillis;
    bool        _wasFaulted;  // detects the no-fault -> fault edge so the fault page shows immediately rather than waiting for the next alternation tick
    bool        _wasCharging; // detects the not-charging -> charging edge, same idea

    // Display watchdog -- see DISPLAY_REINIT_INTERVAL_MS above.
    uint32_t    _lastReinitMillis;
    void        reinitPanel();

    // Partial-redraw sprites (avoid full-screen flicker).
    // IMPORTANT: these are constructed with &_lcd as their parent via the
    // member initializer list in the .cpp file. Do NOT default-construct
    // them and reassign in the constructor body -- LGFX_Sprite's default
    // ctor leaves internal state that is unsafe to overwrite via operator=,
    // and doing so causes a LoadProhibited crash (0xa5a5a5a5 poison pattern)
    // the first time createSprite()/pushSprite() touches the object.
    lgfx::LGFX_Sprite _sprTopBar;
    lgfx::LGFX_Sprite _sprMod0;
    lgfx::LGFX_Sprite _sprMod1;
    lgfx::LGFX_Sprite _sprStatus;
    // Full-screen -- shared by the fault page and the charging page, since
    // only one of the two is ever shown at a time (fault always wins).
    // Reusing one sprite instead of allocating a second ~300KB PSRAM buffer.
    lgfx::LGFX_Sprite _sprFullPage;

    // Accessor so drawModule(idx) can still index like an array
    lgfx::LGFX_Sprite& sprMod(int idx) { return idx == 0 ? _sprMod0 : _sprMod1; }

    void drawTopBar(const DisplayData& d);
    void drawModule(int idx, const DisplayData& d);
    void drawStatusBar(const DisplayData& d);
    void drawFaultPage(const DisplayData& d);
    void drawChargingPage(const DisplayData& d);

    // Helpers
    void drawSoCBox(lgfx::LGFX_Sprite& spr, uint8_t soc);
    void drawCellBlock(lgfx::LGFX_Sprite& spr, int cellIdx,
                       int x, int y, float volt, bool balancing);
    uint16_t socColor(uint8_t soc);
    uint16_t cellColor(float volt);
    uint16_t deltaColor(int deltaMv);
};

// ── SoC estimation (OCV lookup, NMC curve) ───────────────────────────────────
uint8_t estimateSoC(float avgCellVoltage);