# Charge limit tests

How to check the daily (~80 %) limit, the full-charge override and the auto-restart
on the bench, without discharging the pack.

The trick used throughout: **move the target instead of moving the battery.** Setting
the daily target a few tenths of a volt above where the pack is right now gives a
complete, real charge cycle in a few minutes.

**Before you start**

- Flash the `charge-limit-fixes` branch.
- Serial monitor open at 115200 (`pio device monitor`), web UI open on the Charging tab.
- Write down the pack voltage the web UI shows now. Call it **P**.
- Anything typed below in `code` is a serial console command; press Enter after it.

---

## 1. Config check — 1 minute, no charging, no USB

Web UI → Charging → **Charge Target** card. Three rows show the charger's own
configuration, refreshed every 10 seconds:

| Row | Want to see |
|---|---|
| **Charger Curve** | green `CUSTOM_CURVE` — the charger will use your charge target |
| **Auto-restart (RSTE)** | green `enabled` — the charger will restart a cycle on its own |
| **Restart At** | a voltage below the Daily Charge Target |

**If Charger Curve is red** (`PRESET_GEL` / `PRESET_FLOODED` / `PRESET_AGM`) the charger
is ignoring your charge target and running its own lead-acid voltages. That one needs
USB: console `CHGCURVESEL=0`, then `o` twice to latch it.

**If Auto-restart shows `off`:** switch the charger's AC off and on once — SYSTEM_CONFIG
changes only take effect at AC power-up — then reload the page. If it is still `off`
after an AC power-cycle, see the open item at the bottom of this file.

✅ **Pass:** Charger Curve green, Auto-restart `enabled`, Restart At below the target.

---

## 2. Does it stop at the target? — 10 minutes

1. Web UI → Charging → Setpoints → **Daily Charge Target** = **P + 0.3** → Save.
2. Confirm the top of the page: Mode = `Daily limit`, Active Target = your new value.
3. Press **Output ON**.

**Watch for, in order:**

- Stage goes to `CC_STAGE`, output current rises.
- Pack voltage climbs toward the target.
- Stage goes to `CV_STAGE`, then `FULLY_CHARGED`.
- Output current falls to about 0 A.
- **Pack voltage stops at your target and does not go past it.**

✅ **Pass:** it stops at the target by itself. That is the same mechanism that stops
the real charge at 80 % — only the distance travelled is different.

❌ **If the pack voltage sails past the target:** the new backstop trips at
target + 0.50 V — buzzer, `PACK OVER TARGET` on the LCD and fault page, output off.
That means the charger is not obeying `CURVE_CV`; go back to test 1 and check
`CURVE_CONFIG`.

Leave it sitting for 10 minutes and check the voltage is still parked at the target.

> **Getting to `FULLY_CHARGED` quickly.** The charger only declares full when the
> taper current falls below **Curve TC**, and a half-charged 10 kWh pack will happily
> swallow 15 A at a low target for hours. The trick is to set the daily target only
> just above where the pack already sits: the current then starts *below* the taper
> cutoff and it reports full within seconds. On the 2026-09-25 run, a target of
> 21.50 V with the pack at 21.38 V reached `FULLY_CHARGED` in 15 seconds.
>
> **Do not raise Curve TC to force this.** This charger clamps `CURVE_TC` to
> **6.75 A** (30 % of its 22.5 A rating) and silently keeps its old value. The
> read-back check then sees the mismatch and — correctly — switches the output off
> after three strikes. Anything above 6.75 A will do that.

---

## 3. Full-charge override — 10 minutes

This is the one that used to cancel itself within three seconds.

1. **Full-Charge Target** = **P + 0.6** (0.3 V above the daily target) → Save.
2. Leave the pack where test 2 left it: full, at the daily target, Stage `FULLY_CHARGED`.
3. Press **Full-Charge Override: OFF** (it becomes ON).

**Expect within ~3 seconds:**

- Mode = `FULL CHARGE OVERRIDE`, Active Target = the full-charge value.
- Log: `Full-charge override: charging to 24.xx V, will return to the daily target…`
- Current rises again, Stage back to `CC_STAGE`.

🔑 **The key test:** watch the Mode for a full minute. It must **stay** on
`FULL CHARGE OVERRIDE`. Before the fix it flipped back to `Daily limit` within a few
seconds and the pack never charged any further.

**Then let it finish.** When it reaches the full-charge target:

- Log: `Full charge complete -- reverting to daily charge target (24.xx V)`
- Mode returns to `Daily limit` **by itself**, Active Target back to the daily value.

✅ **Pass:** override sticks, charges to the higher target, reverts on its own.

### 3b. The harder case — 2 minutes

1. Press **Output OFF**.
2. Press the override button on.
3. **Wait 30 seconds without touching anything.** Mode must still read `FULL CHARGE OVERRIDE`.
4. Press **Output ON** → it charges to the full-charge target.

✅ **Pass:** the override survives with the charger switched off. (This was the
worst case before the fix — it cancelled itself every time.)

---

## 4. Auto-restart point — 3 minutes

The restart point must always sit below the daily target, or the pack would be charged
non-stop.

1. Web UI → **Auto-restart-charge Voltage** → type a value **above** the daily target
   (e.g. `24.50` when the daily target is `24.00`) → Save.
2. Re-open the Settings/Charging tab.

✅ **Pass:** the value came back as `23.70` (daily target − 0.30), not what you typed.

Same check on the console — `CHGRSTV=24.5` should clamp, and `h` shows the current
maximum:

```
CHGRSTV=24.5
```

Confirm the charger really has it:

```
CHGRAWR=0xB9
```

The reply is hundredths of a volt in hex — `0x0906` = 2310 = **23.10 V**.

**About the real restart:** that the charger *acts* on this needs the pack to genuinely
self-discharge down to the restart point, which no bench test can fake (the clamp in
step 1 exists precisely to stop the pack being left below the restart point). What you
can confirm today is all three preconditions: `RSTE=1` in console `y`, the right value
in `CHG_RST_VBAT` above, and a completed charge cycle with the output left ON. When the
pack does drift down, the Stage goes from `FULLY_CHARGED` back to `CC_STAGE` on its own,
with nothing pressed.

---

## 5. The over-target backstop — 3 minutes

Proves the new protection works, safely, using only a settings change.

1. Start charging again (test 2) so the output is **ON**.
2. Drop the **Daily Charge Target** to **1.0 V below** the current pack voltage → Save.

**Expect within a second or two:**

- Log: `Pack is 24.xx V with a charge target of 23.xx V -- the charger is not holding the target. Switching it off.`
- Output goes OFF, buzzer sounds, fault page and LCD show `PACK OVER TARGET 24.xx V`.

3. Put the Daily Charge Target back to **P + 0.3** → Save. The fault clears by itself
   once the pack is back under the target.
4. The charger stays **off** on purpose — press **Output ON** to carry on.

✅ **Pass:** it tripped, alarmed, and did not restart by itself.

---

## 6. Nothing else broke — 2 minutes

| Do | Expect |
|---|---|
| Console `t` | test fault for 5 s, charger held off, then clears |
| Console `x` | `NO COMMS` fault after 10–15 s, charger off, clears by itself after 40 s |
| Console `y` | `RSTE=1`, `CUSTOM_CURVE`, no warnings underneath |
| Charger ON, then `z` (reboot) | comes back with the output **OFF** (by design) |

---

## Afterwards

Put the real values back:

- **Daily Charge Target** → `24.00` (4.00 V/cell, ~80 %)
- **Full-Charge Target** → `24.90` (4.15 V/cell)
- **Auto-restart-charge Voltage** → `23.10` (3.85 V/cell)

and leave the charger **Output ON** so the auto-restart has something to act on.

---

## Results of the 2026-09-25 run

Run remotely over the HTTP API against the real pack (2 modules, 21.3 V, 51 % SoC).

| Test | Result |
|---|---|
| 1 — config visible | Curve `CUSTOM_CURVE` (0x0084) ✅ · **Auto-restart still `off`** ❌ (see below) |
| 2 — stops at target | ✅ regulated at exactly 21.50 V, `FLOAT_STAGE FULLY_CHARGED`, float held at the target not above it |
| 3 — override sticks | ✅ took effect in 8 s, cleared `FULLY_CHARGED`, resumed at 19.9 A, held for 2 min; reverted to the daily target by itself on the next `FULLY_CHARGED` |
| 3b — override with output off | ✅ held 65 s (note: this charger reports `IDLE`, not `FULLY_CHARGED`, while the output is off, so the stale-full trap does not arise on this path — test 3 is the decisive one) |
| 4 — restart point clamp | ✅ wrote 24.50 V, device stored 23.70 V; live re-clamp to 21.30 V when the target moved to 21.60 V |
| 5 — over-target backstop | ✅ `PACK OVER TARGET 21.52V`, output off in under 3 s, buzzer heard, no auto-resume |
| — read-back protection | ✅ unplanned: `CURVE_TC` 15 A was clamped to 6.75 A by the charger, mismatch caught 3× and the output switched off |

Also confirmed: `FV == CV` is accepted by this charger (`read-back OK: CV=21.500 FV=21.500`),
which had been an open risk — a rejection there would have switched the charger off.

### Closed: RSTE is not implemented on this charger

`SYSTEM_CONFIG` bit 3 is RSTE, confirmed on page 53 of the NPB/NPP CAN manual, and the
NPB-750 spec lists both `SYSTEM_CONFIG` and `CHG_RST_VBAT` as R/W. But the charger
accepts the write frame and silently ignores the bit: `wrote SYSTEM_CONFIG 0x9 but it
still reads 0x1`, five attempts, with the charger online and answering everything else.
The manual's own per-series support row for that register reads mostly `NO`, so RSTE is
documented for the series and not implemented on this model.

The firmware now retries the write every 10 s (up to 5 times) instead of only once at
boot -- that part was a real bug, since the charger's AC comes on independently and is
often absent when the board boots. It gives up with a clear log line rather than
pretending it worked.

**Consequence:** the charger will not resume charging on its own. Automatic top-ups have
to be done by the firmware instead -- see the pack-voltage trigger discussion; the
ESP32 already knows the pack voltage and the threshold and can command the output.
