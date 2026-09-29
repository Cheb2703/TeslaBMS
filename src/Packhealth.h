#pragma once
#include <Arduino.h>
#include "Displaymanager.h"   // DisplayData

class BMSModuleManager;

// ─────────────────────────────────────────────────────────────────────────────
//  Packhealth — charge wiring resistance and pack capacity, from charging data
// ─────────────────────────────────────────────────────────────────────────────
//
//  Charge wiring resistance ("connection check")
//    The charger measures the voltage at its own terminals; the BMS measures
//    the cells. While charging, the difference divided by the current is the
//    resistance of everything in between: cables, lugs, fuse, busbars. A lug
//    working loose shows up as that figure rising, long before it gets hot.
//    The two instruments don't agree exactly even at 0 A, so the difference
//    with the charger switched off and the pack at rest is measured first and
//    subtracted (the "zero offset"). One figure per charge session is kept;
//    a clear rise over the usual value raises a screen-only warning.
//    Only the trend means much: the charger's voltage reading has a coarse
//    resolution (10 mV), and at 20 A that is 0.5 mOhm.
//
//  Capacity
//    The charger reports its current, so the amp-hours put in during a charge
//    are known. The rested average cell voltage just before the charge and at
//    the next power-on (Celldrift's rested reading) give the charge level
//    before and after, from a typical NCA open-circuit-voltage table. Capacity
//    = amp-hours / change in charge level. Only charges that move the level by
//    CAPACITY_MIN_SOC_STEP or more count, and a power-on without a rested
//    reading (lift used straight away) drops that charge. Each estimate is
//    rough; the median of several, and its trend over months, is the useful
//    number. Everything is saved in NVS (namespace "packhealth").

namespace Packhealth {
    void begin();   // setup(), after Blackbox::begin() (needs the power-on number)

    // loop()'s 1-second block, after DisplayData is built.
    //   chargeEvent: from Blackbox::chargerTick() -- 0 none, 1 charge started, 2 stopped
    //   steadyState: 0 = not enough readings yet this power-on, 1 = steady, 2 = moving
    void poll(const DisplayData& dd, int chargeEvent, bool chargerOnline, bool chargerOn,
              float chargerVolts, float chargerAmps, int steadyState, BMSModuleManager& bms);

    void   requestReset();   // web task; done by poll()
    String json();           // web task: for the Charging tab
}
