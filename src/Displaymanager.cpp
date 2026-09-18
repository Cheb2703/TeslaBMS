#include "DisplayManager.h"
#include "ChargerNPB.h" // NPB_CHG_*/NPB_FAULT_* bit macros for decoding the charging page
#include "Logger.h"     // Logger::info() for the display watchdog reinit log line

// ── OCV → SoC lookup table (Panasonic NCR18650B, 11 breakpoints) ────────────
// Rescaled at the top end (3.85V and up) so that 4.00V/cell -- the charger's
// daily-limit target (see CHARGER_INIT_DAILY_V in main.cpp) -- reads 80%, and
// 4.15V/cell -- the full-charge override target -- reads 100%. This matches
// how the pack is actually being managed (Tesla-style ~80% daily / 100% only
// on override) rather than raw cell-physics OCV, which would show 4.00V as
// ~92%. Below 3.70V is left as true OCV -- it doesn't affect the daily/full
// decision and matters more for low-charge warnings, so no reason to distort it.
static const float OCV_VOLT[] = { 3.00f, 3.10f, 3.20f, 3.30f, 3.40f,
                                   3.50f, 3.60f, 3.70f, 3.85f, 4.00f, 4.15f };
static const uint8_t OCV_SOC[] = { 0,   3,   7,  12,  28,
                                    42,  55,  68,  74,  80, 100 };
static const int OCV_POINTS = 11;

uint8_t estimateSoC(float v) {
    if (v <= OCV_VOLT[0])              return 0;
    if (v >= OCV_VOLT[OCV_POINTS - 1]) return 100;
    for (int i = 1; i < OCV_POINTS; i++) {
        if (v <= OCV_VOLT[i]) {
            float t = (v - OCV_VOLT[i-1]) / (OCV_VOLT[i] - OCV_VOLT[i-1]);
            return (uint8_t)(OCV_SOC[i-1] + t * (OCV_SOC[i] - OCV_SOC[i-1]));
        }
    }
    return 100;
}

// Formats a duration in seconds as "Ns", "MmSs", or "HhMm" depending on
// magnitude, so durations don't turn into a huge, unreadable seconds count
// once a fault (or the time since one cleared) runs past a minute or hour.
static void formatDurationSec(uint32_t totalSeconds, char* buf, size_t bufSize) {
    if (totalSeconds < 60) {
        snprintf(buf, bufSize, "%lus", (unsigned long)totalSeconds);
    } else if (totalSeconds < 3600) {
        uint32_t m = totalSeconds / 60;
        uint32_t s = totalSeconds % 60;
        snprintf(buf, bufSize, "%lum%lus", (unsigned long)m, (unsigned long)s);
    } else {
        uint32_t h = totalSeconds / 3600;
        uint32_t m = (totalSeconds % 3600) / 60;
        snprintf(buf, bufSize, "%luh%lum", (unsigned long)h, (unsigned long)m);
    }
}

// ── Constructor ───────────────────────────────────────────────────────────────
DisplayManager::DisplayManager()
    : _ready(false),
      _currentPage(PAGE_DASHBOARD),
      _pageEnteredMillis(0),
      _wasFaulted(false),
      _wasCharging(false),
      _sprTopBar(&_lcd),
      _sprMod0(&_lcd),
      _sprMod1(&_lcd),
      _sprStatus(&_lcd),
      _sprFullPage(&_lcd)
{
}

void DisplayManager::begin() {
    _lcd.init();
    _lcd.setRotation(1);
    _lcd.setBrightness(200);
    _lcd.fillScreen(COL_BG);

    Serial.print("PSRAM found: ");
    Serial.println(psramFound() ? "yes" : "no");
    Serial.printf("Free heap before sprites: %u bytes\n", ESP.getFreeHeap());
    if (psramFound()) {
        Serial.printf("Free PSRAM before sprites: %u bytes\n", ESP.getFreePsram());
    }

    _sprTopBar.setPsram(true);
    _sprTopBar.setColorDepth(16);
    _sprTopBar.createSprite(DISP_W, TOPBAR_H + 2);

    _sprMod0.setPsram(true);
    _sprMod0.setColorDepth(16);
    _sprMod0.createSprite(MOD_W, MOD_H);

    _sprMod1.setPsram(true);
    _sprMod1.setColorDepth(16);
    _sprMod1.createSprite(MOD_W, MOD_H);

    _sprStatus.setPsram(true);
    _sprStatus.setColorDepth(16);
    _sprStatus.createSprite(DISP_W, DISP_H - STATUS_Y);

    _sprFullPage.setPsram(true);
    _sprFullPage.setColorDepth(16);
    _sprFullPage.createSprite(DISP_W, DISP_H);

    Serial.printf("Free heap after sprites: %u bytes\n", ESP.getFreeHeap());
    if (psramFound()) {
        Serial.printf("Free PSRAM after sprites: %u bytes\n", ESP.getFreePsram());
    }

    _lcd.drawFastHLine(0, MOD_Y - 2, DISP_W, COL_BORDER);
    _lcd.drawFastHLine(0, STATUS_Y - 2, DISP_W, COL_BORDER);

    _lastReinitMillis = millis();
    _ready = true;
}

// Display watchdog: manually pulse RST (GPIO9), then resend just the
// ST7796S init command list -- see Panel_ST7796_CmdReinit in the header for
// why this deliberately does NOT call _lcd.init() again (confirmed against
// the actual LovyanGFX v1 source: Panel_Device::init() unconditionally
// re-attaches the backlight's LEDC channel via ledcAttach(), which
// Arduino-ESP32's own core does not support calling twice on an
// already-attached pin -- this is what broke the display on the bench).
// The SPI bus and backlight are left completely untouched here; only the
// panel controller itself gets reset and re-commanded.
//
// Deliberately NOT touching _currentPage / _pageEnteredMillis / _wasFaulted
// / _wasCharging here: every page type (dashboard, fault, charging) gets
// fully redrawn on every single update() call regardless of _currentPage --
// the fault and charging pages each fill a full-screen sprite from scratch
// every call, and the dashboard's partial sprites get repainted every call
// too. So the very next update() after this reinit repaints whatever's
// actually current correctly on its own.
void DisplayManager::reinitPanel() {
    uint32_t t0 = millis();

    // Optional cosmetic step: drop backlight to hide the reset's undefined-
    // GRAM flash (shows as white on this panel) instead of showing it.
    // setBrightness() only calls ledcWrite() on the already-attached
    // channel -- NOT ledcAttach() -- so unlike _lcd.init(), this is safe to
    // call repeatedly.
    _lcd.setBrightness(0);

    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
    delayMicroseconds(20);
    digitalWrite(9, HIGH);
    delay(120); // ST7796S: minimum wait after reset release before commands

    _lcd.resendInitCommandsOnly();
    _lcd.setRotation(1); // the raw command list doesn't set MADCTL -- the
                          // library normally reapplies this after init(),
                          // which we're bypassing here, so do it ourselves
                          // or the panel comes back in its default orientation

    _lcd.fillScreen(COL_BG);
    _lcd.drawFastHLine(0, MOD_Y - 2, DISP_W, COL_BORDER);
    _lcd.drawFastHLine(0, STATUS_Y - 2, DISP_W, COL_BORDER);
    _lcd.setBrightness(200); // restore -- content is real again now

    uint32_t elapsed = millis() - t0;
    Logger::info("Display watchdog: proactive panel reinit (GPIO9 RST, command-list-only) took %d ms", (int)elapsed);
    _lastReinitMillis = millis();
}

void DisplayManager::update(const DisplayData& d) {
    if (!_ready) return;
    uint32_t now = millis();

    if (now - _lastReinitMillis >= DISPLAY_REINIT_INTERVAL_MS) {
        reinitPanel();
    }

    // Charging is only relevant when there's actually a charger to report on
    // and it's got its output on right now. This is deliberately independent
    // of charge STAGE (CC/CV/float/full) -- even float/maintain is still
    // worth a "charger is on, here's what it's doing" page.
    bool isCharging = d.chargerPresent && d.chargerOnline && d.chargerOutputOn;

    // Helper to wipe full-screen page residue out of the gaps the
    // dashboard's partial-redraw sprites don't cover.
    auto resetToDashboardBackground = [&]() {
        _lcd.fillScreen(COL_BG);
        _lcd.drawFastHLine(0, MOD_Y - 2, DISP_W, COL_BORDER);
        _lcd.drawFastHLine(0, STATUS_Y - 2, DISP_W, COL_BORDER);
    };

    auto drawDashboard = [&]() {
        drawTopBar(d);
        drawModule(0, d);
        drawModule(1, d);
        drawStatusBar(d);
    };

    if (!d.isFaulted && !isCharging) {
        // Nothing active -- always show the dashboard.
        if (_currentPage != PAGE_DASHBOARD) {
            resetToDashboardBackground();
            _currentPage = PAGE_DASHBOARD;
        }
        _wasFaulted  = false;
        _wasCharging = false;
        drawDashboard();
        return;
    }

    // Catch the fault->cleared edge BEFORE the branches below touch
    // _wasFaulted, so a stale PAGE_FAULT can't linger on screen reporting a
    // fault that's no longer active while we wait for the charging page's
    // own "just started" edge to catch up.
    bool faultJustCleared = _wasFaulted && !d.isFaulted;

    if (d.isFaulted) {
        // Fault always wins over the charging page -- more urgent to see.
        if (!_wasFaulted) {
            // Just started faulting -- show the fault page right away
            // instead of waiting up to FAULT_PAGE_INTERVAL_MS.
            _currentPage = PAGE_FAULT;
            _pageEnteredMillis = now;
            _wasFaulted = true;
        } else if (now - _pageEnteredMillis >= FAULT_PAGE_INTERVAL_MS) {
            // Already faulted for a while -- alternate between the fault
            // page and the dashboard so live pack data is still visible.
            _currentPage = (_currentPage == PAGE_FAULT) ? PAGE_DASHBOARD : PAGE_FAULT;
            _pageEnteredMillis = now;
            if (_currentPage == PAGE_DASHBOARD) resetToDashboardBackground();
        }
        // Don't let charging-page alternation run concurrently -- just
        // track that it's active so the edge fires cleanly once we get here.
        _wasCharging = isCharging;
    } else {
        // Not faulted. isCharging must be true to have reached this branch.
        _wasFaulted = false;
        if (!_wasCharging || faultJustCleared) {
            // Either just started charging, or a fault just cleared and
            // charging was (or still is) active -- show the charging page
            // immediately rather than leaving the fault page up stale.
            _currentPage = PAGE_CHARGING;
            _pageEnteredMillis = now;
            _wasCharging = true;
        } else if (now - _pageEnteredMillis >= CHARGE_PAGE_INTERVAL_MS) {
            _currentPage = (_currentPage == PAGE_CHARGING) ? PAGE_DASHBOARD : PAGE_CHARGING;
            _pageEnteredMillis = now;
            if (_currentPage == PAGE_DASHBOARD) resetToDashboardBackground();
        }
    }

    switch (_currentPage) {
        case PAGE_FAULT:     drawFaultPage(d);    break;
        case PAGE_CHARGING:  drawChargingPage(d); break;
        default:              drawDashboard();     break;
    }
}

// ── Top bar ───────────────────────────────────────────────────────────────────
void DisplayManager::drawTopBar(const DisplayData& d) {
    auto& s = _sprTopBar;
    s.fillSprite(COL_BG);

    drawSoCBox(s, d.socPercent);

    s.drawFastVLine(SOC_BOX_W + 8, 4, TOPBAR_H - 8, COL_BORDER);

    int rx = SOC_BOX_W + 18;
    int col_w = (DISP_W - rx - 4) / 5;

    int deltaMv = (int)((d.cellHigh - d.cellLow) * 1000.0f + 0.5f);

    struct { const char* lbl; uint16_t col; } stats[3] = {
        { "PACK V",   COL_GREEN  },
        { "CELL LOW", COL_WHITE  },
        { "CELL HI",  COL_WHITE  },
    };

    char packStr[12], lowStr[10], hiStr[10], deltaStr[10], tempStr[10];
    snprintf(packStr,  sizeof(packStr),  "%.2f",  d.packVoltage);
    snprintf(lowStr,   sizeof(lowStr),   "%.3f",  d.cellLow);
    snprintf(hiStr,    sizeof(hiStr),    "%.3f",  d.cellHigh);
    snprintf(deltaStr, sizeof(deltaStr), "%dmV",  deltaMv);
    snprintf(tempStr,  sizeof(tempStr),  "%.1fC", d.avgTemp);

    const char* vals[3] = { packStr, lowStr, hiStr };

    // Columns 1-3: PACK V, CELL LOW, CELL HI
    for (int i = 0; i < 3; i++) {
        int cx = rx + i * col_w + col_w / 2;
        s.setTextColor(COL_MUTED);
        s.setTextDatum(lgfx::TC_DATUM);
        s.setTextSize(1);
        s.drawString(stats[i].lbl, cx, 12);
        s.setTextColor(stats[i].col);
        s.setTextSize(2);
        s.drawString(vals[i], cx, 28);
    }

    // Column 4: Delta — "PACK" + hollow triangle + "V" label, mV value below
    int dcx = rx + 3 * col_w + col_w / 2;
    // Centre the whole "PACK [tri] V" group around dcx
    // "PACK " at size 1 = ~30px, triangle 8px, " V" ~8px = ~46px total, start at dcx-23
    int lblStartX = dcx - 23;
    int triY = 12;
    s.setTextColor(COL_MUTED);
    s.setTextSize(1);
    s.setTextDatum(lgfx::TL_DATUM);
    s.drawString("PACK", lblStartX, triY);
    int triX = lblStartX + 28;
    s.drawTriangle(triX + 4, triY,
                   triX,     triY + 7,
                   triX + 8, triY + 7,
                   COL_MUTED);
    s.drawString("V", triX + 10, triY);
    s.setTextColor(deltaColor(deltaMv));
    s.setTextSize(2);
    s.setTextDatum(lgfx::TC_DATUM);
    s.drawString(deltaStr, dcx, 28);

    // Column 5: AVG TEMP
    int tcx = rx + 4 * col_w + col_w / 2;
    s.setTextColor(COL_MUTED);
    s.setTextDatum(lgfx::TC_DATUM);
    s.setTextSize(1);
    s.drawString("AVG TEMP", tcx, 12);
    s.setTextColor(COL_AMBER);
    s.setTextSize(2);
    s.drawString(tempStr, tcx, 28);

    s.pushSprite(0, TOPBAR_Y);
}

// ── SoC box ───────────────────────────────────────────────────────────────────
void DisplayManager::drawSoCBox(lgfx::LGFX_Sprite& s, uint8_t soc) {
    uint16_t col = socColor(soc);

    uint16_t bgCol;
    if      (col == COL_GREEN) bgCol = 0x0421;
    else if (col == COL_AMBER) bgCol = 0x2200;
    else                       bgCol = 0x2000;

    s.fillRoundRect(SOC_BOX_X, 0, SOC_BOX_W, SOC_BOX_H, 4, bgCol);
    s.drawRoundRect(SOC_BOX_X, 0, SOC_BOX_W, SOC_BOX_H, 4, col);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", soc);
    s.setTextDatum(lgfx::MR_DATUM);
    s.setTextColor(COL_WHITE);
    s.setTextSize(4);
    s.drawString(buf, SOC_BOX_X + SOC_BOX_W - 22, SOC_BOX_H / 2 - 4);

    s.setTextSize(2);
    s.setTextDatum(lgfx::ML_DATUM);
    s.setTextColor(COL_WHITE);
    s.drawString("%", SOC_BOX_X + SOC_BOX_W - 20, SOC_BOX_H / 2 - 10);

    s.setTextSize(1);
    s.setTextColor(COL_WHITE);
    s.setTextDatum(lgfx::BC_DATUM);
    s.drawString("SoC EST", SOC_BOX_X + SOC_BOX_W / 2, SOC_BOX_H - 3);
}

// ── Module panel ──────────────────────────────────────────────────────────────
void DisplayManager::drawModule(int idx, const DisplayData& d) {
    auto& s = sprMod(idx);
    s.fillSprite(COL_BG);

    s.fillRoundRect(0, 0, MOD_W, MOD_H, 4, COL_SURFACE);
    s.drawRoundRect(0, 0, MOD_W, MOD_H, 4, COL_BORDER);

    // Module title: "MODULE X" in white, voltage in color
    s.setTextSize(2);
    s.setTextDatum(lgfx::TL_DATUM);
    char modLabel[12];
    snprintf(modLabel, sizeof(modLabel), "MODULE %d ", idx + 1);
    s.setTextColor(COL_WHITE);
    s.drawString(modLabel, 8, 10);

    char voltLabel[12];
    snprintf(voltLabel, sizeof(voltLabel), "%.2fV", d.moduleVolt[idx]);
    s.setTextColor(cellColor(d.moduleVolt[idx] / 6.0f));
    s.drawString(voltLabel, s.textWidth(modLabel) + 40, 10);

    // 3×2 cell grid
    const int CELL_W = 68;
    const int CELL_H = 54;
    const int GRID_X = 6;
    const int GRID_Y = 32;
    const int GAP    = 4;

    for (int i = 0; i < 6; i++) {
        int col = i % 3;
        int row = i / 3;
        int cx  = GRID_X + col * (CELL_W + GAP);
        int cy  = GRID_Y + row * (CELL_H + GAP);
        drawCellBlock(s, idx * 6 + i, cx, cy, d.cellVolt[idx][i], d.cellBalancing[idx][i]);
    }

    // ── Temp + delta area ────────────────────────────────────────────────────
    // Left half: NEG stacked above POS (your layout from doc 22)
    // Right half: ΔV label + value, color-coded by spread size
    int tempY = GRID_Y + 2 * (CELL_H + GAP) + 6;

    // Calculate per-module delta from raw cell voltages
    float modLow  = d.cellVolt[idx][0];
    float modHigh = d.cellVolt[idx][0];
    for (int c = 1; c < 6; c++) {
        if (d.cellVolt[idx][c] < modLow)  modLow  = d.cellVolt[idx][c];
        if (d.cellVolt[idx][c] > modHigh) modHigh = d.cellVolt[idx][c];
    }
    int modDeltaMv = (int)((modHigh - modLow) * 1000.0f + 0.5f);

    // Left: stacked temperatures
    s.setTextDatum(lgfx::TL_DATUM);
    s.setTextSize(2);
    s.setTextColor(COL_MUTED);
    s.drawString("NEG:", 6, tempY + 2);
    s.drawString("POS:", 6, tempY + 25);

    char tBuf[12];
    snprintf(tBuf, sizeof(tBuf), "%.1fC", d.tempNeg[idx]);
    s.setTextColor(COL_AMBER);
    s.drawString(tBuf, 54, tempY + 2);

    snprintf(tBuf, sizeof(tBuf), "%.1fC", d.tempPos[idx]);
    s.setTextColor(COL_AMBER);
    s.drawString(tBuf, 54, tempY + 25);

    // Right: per-module delta — label row aligns with NEG, value row aligns with POS
    // Triangle drawn as the delta symbol, followed by "V" then the mV value inline
    int dx = MOD_W / 2 + 10;
    uint16_t dCol = deltaColor(modDeltaMv);

    // Row 1: triangle (14x12px to match size-2 font) + "V" label
    int triX = dx + 5;
    int triY = tempY + 2;
    s.drawTriangle(triX + 7, triY,           // apex
                   triX,     triY + 12,      // bottom-left
                   triX + 14, triY + 12,     // bottom-right
                   COL_MUTED);
    s.setTextColor(COL_MUTED);
    s.setTextSize(2);
    s.setTextDatum(lgfx::TL_DATUM);
    s.drawString("V", triX + 16, triY);

    // Row 1 value: delta mV inline to the right — same y as NEG value
    char dBuf[10];
    snprintf(dBuf, sizeof(dBuf), "%dmV", modDeltaMv);
    s.setTextColor(dCol);
    s.setTextSize(2);
    s.drawString(dBuf, dx + 40, tempY + 2);

    int destX = (idx == 0) ? MOD1_X : MOD2_X;
    s.pushSprite(destX, MOD_Y);
}

// ── Cell block ────────────────────────────────────────────────────────────────
void DisplayManager::drawCellBlock(lgfx::LGFX_Sprite& s, int cellIdx,
                                    int x, int y, float volt, bool balancing) {
    uint16_t col = cellColor(volt);
    uint16_t bgCol = balancing ? 0x2A10 : 0x10A2;

    s.fillRoundRect(x, y, 68, 54, 3, bgCol);
    if (balancing)
        s.drawRoundRect(x, y, 68, 54, 3, COL_AMBER);
    else
        s.drawRoundRect(x, y, 68, 54, 3, COL_BORDER);

    char cLabel[6];
    snprintf(cLabel, sizeof(cLabel), "C%d", cellIdx + 1);
    s.setTextColor(COL_MUTED);
    s.setTextSize(1);
    s.setTextDatum(lgfx::TL_DATUM);
    s.drawString(cLabel, x + 4, y + 3);

    if (balancing) {
        s.fillCircle(x + 61, y + 5, 3, COL_AMBER);
    }

    char vBuf[10];
    snprintf(vBuf, sizeof(vBuf), "%.3f", volt);
    s.setTextColor(col);
    s.setTextSize(2);
    s.setTextDatum(lgfx::MC_DATUM);
    s.drawString(vBuf, x + 34, y + 26);

    int barX = x + 4;
    int barY = y + 44;
    int barW = 60;
    int barH = 3;
    s.fillRect(barX, barY, barW, barH, COL_BORDER);
    float pct = (volt - 3.0f) / (4.15f - 3.0f);
    pct = pct < 0.0f ? 0.0f : (pct > 1.0f ? 1.0f : pct);
    int filled = (int)(barW * pct);
    if (filled > 0) s.fillRect(barX, barY, filled, barH, col);
}

// ── Status bar ────────────────────────────────────────────────────────────────
void DisplayManager::drawStatusBar(const DisplayData& d) {
    auto& s = _sprStatus;
    s.fillSprite(COL_BG);

    // This can now run even while faulted (during the dashboard half of the
    // 8-second alternation started in update()), so the pill has to reflect
    // the actual current state rather than always showing OK.
    if (d.isFaulted) {
        s.fillRoundRect(4, 4, 100, 18, 4, 0x2000);
        s.drawRoundRect(4, 4, 100, 18, 4, COL_RED);
        s.setTextColor(COL_RED);
        s.setTextSize(1);
        s.setTextDatum(lgfx::ML_DATUM);
        s.drawString("! FAULT", 12, 13);
    } else {
        s.fillRoundRect(4, 4, 100, 18, 4, 0x0421);
        s.drawRoundRect(4, 4, 100, 18, 4, COL_GREEN);
        s.setTextColor(COL_GREEN);
        s.setTextSize(1);
        s.setTextDatum(lgfx::ML_DATUM);
        s.drawString("ALL OK", 12, 13);
    }

    // Centre-left: while faulted, point at the fault page rather than
    // showing history (the fault isn't over yet); once clear, fall back to
    // fault history / no-history text as before.
    char faultBuf[64];
    if (d.isFaulted) {
        snprintf(faultBuf, sizeof(faultBuf), "%d ACTIVE -- SEE FAULT PAGE", d.activeFaultCount);
        s.setTextColor(COL_RED);
    } else if (d.hasFaultHistory) {
        uint32_t durSec = d.lastFaultDurationMs / 1000;
        char durPart[16];
        if (durSec == 0) {
            snprintf(durPart, sizeof(durPart), "%lums", (unsigned long)d.lastFaultDurationMs);
        } else {
            formatDurationSec(durSec, durPart, sizeof(durPart));
        }

        char agoPart[16];
        formatDurationSec(d.secondsSinceFaultCleared, agoPart, sizeof(agoPart));

        if (d.lastFaultCount > 1) {
            snprintf(faultBuf, sizeof(faultBuf), "LAST: %s +%d, %s, %s AGO",
                     d.lastFaultReason, d.lastFaultCount - 1, durPart, agoPart);
        } else {
            snprintf(faultBuf, sizeof(faultBuf), "LAST: %s, %s, %s AGO",
                     d.lastFaultReason, durPart, agoPart);
        }
        s.setTextColor(COL_AMBER);
    } else {
        snprintf(faultBuf, sizeof(faultBuf), "NO FAULTS RECORDED");
        s.setTextColor(COL_DIM);
    }
    s.setTextDatum(lgfx::ML_DATUM);
    s.drawString(faultBuf, 112, 13);

    if (d.balancingCount > 0) {
        char balBuf[20];
        snprintf(balBuf, sizeof(balBuf), "BAL %d", d.balancingCount);
        s.fillRoundRect(280, 4, 80, 18, 4, 0x2A00);
        s.drawRoundRect(280, 4, 80, 18, 4, COL_AMBER);
        s.setTextColor(COL_AMBER);
        s.setTextDatum(lgfx::ML_DATUM);
        s.drawString(balBuf, 288, 13);
    }

    uint32_t sec = d.uptimeSeconds;
    uint32_t hh  = sec / 3600;
    uint32_t mm  = (sec % 3600) / 60;
    uint32_t ss  = sec % 60;
    char uptBuf[16];
    snprintf(uptBuf, sizeof(uptBuf), "UP %02lu:%02lu:%02lu", hh, mm, ss);
    s.setTextColor(COL_DIM);
    s.setTextDatum(lgfx::MR_DATUM);
    s.drawString(uptBuf, DISP_W - 4, 13);

    s.pushSprite(0, STATUS_Y);
}

// ── Fault page ────────────────────────────────────────────────────────────────
// Shown instead of the normal dashboard whenever ANY fault is active (see
// update()). Lists every simultaneously-active fault with a running
// duration, so nothing gets hidden behind whichever fault happened to be
// checked last -- no serial connection needed to see what's going on.
void DisplayManager::drawFaultPage(const DisplayData& d) {
    auto& s = _sprFullPage;
    s.fillSprite(COL_BG);

    // Header
    s.fillRect(0, 0, DISP_W, 50, 0x2000);
    s.drawFastHLine(0, 50, DISP_W, COL_RED);
    char headerBuf[32];
    snprintf(headerBuf, sizeof(headerBuf), "ACTIVE FAULTS: %d", d.activeFaultCount);
    s.setTextColor(COL_RED);
    s.setTextSize(3);
    s.setTextDatum(lgfx::ML_DATUM);
    s.drawString(headerBuf, 12, 25);

    // One row per active fault (up to MAX_DISPLAY_FAULTS)
    int shown = d.activeFaultCount < MAX_DISPLAY_FAULTS ? d.activeFaultCount : MAX_DISPLAY_FAULTS;
    const int rowH = 42;
    const int firstRowY = 62;

    for (int i = 0; i < shown; i++) {
        int y = firstRowY + i * rowH;

        s.fillCircle(20, y + 12, 5, COL_RED);

        s.setTextColor(COL_WHITE);
        s.setTextSize(2);
        s.setTextDatum(lgfx::ML_DATUM);
        s.drawString(d.faultReasons[i], 36, y + 8);

        uint32_t durMs = d.faultDurationsMs[i];
        char durBuf[16];
        uint32_t durSec = durMs / 1000;
        formatDurationSec(durSec, durBuf, sizeof(durBuf));
        s.setTextColor(COL_AMBER);
        s.setTextSize(1);
        s.setTextDatum(lgfx::ML_DATUM);
        s.drawString(durBuf, 36, y + 28);

        if (i < shown - 1) s.drawFastHLine(12, y + rowH - 4, DISP_W - 24, COL_BORDER);
    }

    // If there are more active faults than fit on screen, say so rather
    // than silently hiding them.
    if (d.activeFaultCount > MAX_DISPLAY_FAULTS) {
        char moreBuf[40];
        snprintf(moreBuf, sizeof(moreBuf), "+%d more -- see serial console", d.activeFaultCount - MAX_DISPLAY_FAULTS);
        s.setTextColor(COL_MUTED);
        s.setTextSize(1);
        s.setTextDatum(lgfx::ML_DATUM);
        s.drawString(moreBuf, 12, DISP_H - 14);
    }

    s.pushSprite(0, 0);
}

// ── Charging page ─────────────────────────────────────────────────────────────
// Shown instead of the normal dashboard while the charger is on (see
// update() -- subordinate to the fault page, which always wins). Gives a
// full status readout of the MEAN WELL NPB-750-24: measured V/I, commanded
// setpoints, stage (CC/CV/float/full), internal temp, charger-side fault
// bits, comms freshness, and a rough time-to-full estimate.
void DisplayManager::drawChargingPage(const DisplayData& d) {
    auto& s = _sprFullPage;
    s.fillSprite(COL_BG);

    // ── Decode charge stage from CHG_STATUS bits ────────────────────────────
    uint16_t c = d.chargerChgStatusRaw;
    const char* stageTxt;
    uint16_t    stageCol;
    if (c & NPB_CHG_FULLM) {
        stageTxt = "FULL - MAINTAINING"; stageCol = COL_GREEN;
    } else if (c & NPB_CHG_WAKEUP_STOP) {
        stageTxt = "WAKING UP";          stageCol = COL_MUTED;
    } else if (c & NPB_CHG_FVM) {
        stageTxt = "FLOAT";              stageCol = COL_GREEN;
    } else if (c & NPB_CHG_CVM) {
        stageTxt = "CONSTANT VOLTAGE";   stageCol = COL_BLUE;
    } else if (c & NPB_CHG_CCM) {
        stageTxt = "CONSTANT CURRENT";   stageCol = COL_AMBER;
    } else {
        stageTxt = "IDLE";               stageCol = COL_MUTED;
    }
    bool fullyCharged = (c & NPB_CHG_FULLM) != 0;

    // ── Header ───────────────────────────────────────────────────────────────
    s.fillRect(0, 0, DISP_W, 50, 0x0421);
    s.drawFastHLine(0, 50, DISP_W, COL_GREEN);
    s.setTextColor(COL_GREEN);
    s.setTextSize(3);
    s.setTextDatum(lgfx::ML_DATUM);
    s.drawString("CHARGING", 12, 25);

    // Comms-freshness pill, top-right of header
    bool stale = d.chargerLastRxAgoMs > 6000; // no good frame in 2x the 3s poll period
    uint16_t commCol = stale ? COL_RED : COL_GREEN;
    s.fillRoundRect(DISP_W - 130, 14, 118, 24, 4, stale ? 0x2000 : 0x0421);
    s.drawRoundRect(DISP_W - 130, 14, 118, 24, 4, commCol);
    s.setTextColor(commCol);
    s.setTextSize(1);
    s.setTextDatum(lgfx::MC_DATUM);
    s.drawString(stale ? "COMMS STALE" : "CAN OK", DISP_W - 71, 26);

    // ── Stat card grid: 3 columns x 2 rows ──────────────────────────────────
    const int gridX = 12, gridY = 62;
    const int cardW = (DISP_W - gridX * 2 - 2 * 10) / 3;
    const int cardH = 78;
    const int gapX  = 10, gapY = 10;

    auto card = [&](int col, int row, const char* label, const char* value, uint16_t valCol) {
        int x = gridX + col * (cardW + gapX);
        int y = gridY + row * (cardH + gapY);
        s.fillRoundRect(x, y, cardW, cardH, 4, COL_SURFACE);
        s.drawRoundRect(x, y, cardW, cardH, 4, COL_BORDER);
        s.setTextColor(COL_MUTED);
        s.setTextSize(1);
        s.setTextDatum(lgfx::TL_DATUM);
        s.drawString(label, x + 8, y + 8);
        s.setTextColor(valCol);
        s.setTextSize(3);
        s.setTextDatum(lgfx::ML_DATUM);
        s.drawString(value, x + 8, y + cardH / 2 + 12);
    };

    char voutBuf[16], ioutBuf[16], tempBuf[16];
    snprintf(voutBuf, sizeof(voutBuf), "%.2fV", d.chargerVout);
    snprintf(ioutBuf, sizeof(ioutBuf), "%.2fA", d.chargerIout);
    snprintf(tempBuf, sizeof(tempBuf), "%.1fC", d.chargerTemp);

    card(0, 0, "OUTPUT VOLTAGE", voutBuf, COL_WHITE);
    card(1, 0, "OUTPUT CURRENT", ioutBuf, COL_WHITE);
    card(2, 0, "CHARGER TEMP",   tempBuf, (d.chargerTemp >= 60.0f) ? COL_AMBER : COL_WHITE);

    char setpBuf[24];
    snprintf(setpBuf, sizeof(setpBuf), "%.1fV / %.1fA", d.chargerVoltSetpoint, d.chargerCurrSetpoint);

    // Rough linear ETA: remaining Ah at the assumed pack capacity, divided by
    // present output current. Deliberately simple -- see PACK_CAPACITY_AH's
    // comment in DisplayManager.h for the caveats (optimistic during CV/float
    // taper, meaningless once fully charged or if current is ~0).
    char etaBuf[16];
    if (fullyCharged) {
        snprintf(etaBuf, sizeof(etaBuf), "DONE");
    } else if (d.chargerIout < 0.05f) {
        snprintf(etaBuf, sizeof(etaBuf), "--");
    } else {
        float remainAh = (100.0f - (float)d.socPercent) / 100.0f * PACK_CAPACITY_AH;
        float etaHours  = remainAh / d.chargerIout;
        uint32_t etaMin = (uint32_t)(etaHours * 60.0f + 0.5f);
        if (etaMin >= 60) {
            snprintf(etaBuf, sizeof(etaBuf), "%luh%02lum", (unsigned long)(etaMin / 60), (unsigned long)(etaMin % 60));
        } else {
            snprintf(etaBuf, sizeof(etaBuf), "%lum", (unsigned long)etaMin);
        }
    }

    card(0, 1, "STAGE",          stageTxt, stageCol);
    card(1, 1, "SETPOINT V / I", setpBuf,  COL_MUTED);
    card(2, 1, "EST. TIME TO FULL", etaBuf, fullyCharged ? COL_GREEN : COL_WHITE);

    // ── Footer status line: charger-side fault + SoC recap ──────────────────
    int footY = gridY + 2 * (cardH + gapY) + 4;
    s.drawFastHLine(gridX, footY, DISP_W - gridX * 2, COL_BORDER);

    s.setTextSize(1);
    s.setTextDatum(lgfx::ML_DATUM);
    if (d.chargerFaulted) {
        char fb[56];
        snprintf(fb, sizeof(fb), "CHARGER FAULT: %s", d.chargerFaultStr);
        s.setTextColor(COL_RED);
        s.drawString(fb, gridX, footY + 16);
    } else {
        s.setTextColor(COL_DIM);
        s.drawString("CHARGER: NO FAULTS", gridX, footY + 16);
    }

    char socBuf[32];
    snprintf(socBuf, sizeof(socBuf), "PACK SoC %d%%  (%.2fV)", d.socPercent, d.packVoltage);
    s.setTextColor(COL_MUTED);
    s.setTextDatum(lgfx::MR_DATUM);
    s.drawString(socBuf, DISP_W - gridX, footY + 16);

    s.pushSprite(0, 0);
}

// ── Helpers ───────────────────────────────────────────────────────────────────
uint16_t DisplayManager::socColor(uint8_t soc) {
    if (soc >= 25) return COL_GREEN;
    if (soc >= 10) return COL_AMBER;
    return COL_RED;
}

uint16_t DisplayManager::cellColor(float volt) {
    return socColor(estimateSoC(volt));
}

// Delta thresholds: <=20mV green (well balanced), <=50mV amber, >50mV red
uint16_t DisplayManager::deltaColor(int deltaMv) {
    if (deltaMv <= 20) return COL_GREEN;
    if (deltaMv <= 50) return COL_AMBER;
    return COL_RED;
}