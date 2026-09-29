#pragma once
#include <Arduino.h>
#include "Displaymanager.h"   // DisplayData

class BMSModuleManager;

// ─────────────────────────────────────────────────────────────────────────────
//  Celldrift — the self-discharge (cell drift) warning
// ─────────────────────────────────────────────────────────────────────────────
//
//  A cell with a small internal short slowly drains itself, so over time it
//  falls behind the other cells. Tesla watches for exactly this in parked cars;
//  it is an early sign of a failing, possibly dangerous cell.
//
//  This board has no clock and is off most of the time, so drift is measured
//  per power-on, not per day: once per power-on, on the first steady reading
//  with the charger off (a rested pack), each cell's offset from the average
//  of all cells is saved in NVS. The newest reading is compared with earlier
//  ones taken at a similar charge level, skipping any cell that was balanced
//  in between (balancing lowers a cell on purpose). A cell DRIFT_WARN_MV below
//  its usual offset raises a warning: LCD, web and black box, no buzzer, and
//  charging stays allowed. The thresholds are in config.h.
//
//  The warning is saved too, so it is shown again at the next power-on even if
//  that power-on never gets a rested reading. After a cell or module has been
//  dealt with, "Reset drift history" on the web Faults tab starts over.

namespace Celldrift {
    void begin();   // setup(): load the saved history

    // loop()'s 1-second block, after DisplayData is built. steadyState:
    // 0 = not enough readings yet this power-on, 1 = steady, 2 = moving.
    void poll(const DisplayData& dd, int modulesExpected, int steadyState, bool chargerOn, BMSModuleManager& bms);

    // This power-on's rested reading (average cell voltage), once it has been
    // taken. Packhealth uses it as the "after" point of a capacity estimate.
    bool   restedMeanV(float& meanV);

    void   requestReset();   // web task; done by poll()
    String json();           // web task: the last evaluation, for the Faults tab
}
