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

## 1. Boot check — 2 minutes, no charging

Reboot the board (console `z`) and read the boot log.

**Expect three lines:**

| Line | What it means |
|---|---|
| `Charger auto-restart: RSTE enabled (SYSTEM_CONFIG 0x…. -> 0x….)` | the auto-restart was off and has just been switched on |
| `Charger curve config: CURVE_CONFIG=0x…. (CUSTOM_CURVE CURVE_ENABLED …)` | the charger will use our charge target |
| `Saved auto-restart point 24.60 V is not below the daily target 24.00 V — saving 23.70 V instead.` | the old bad restart point being corrected, once |

**If `CURVE_CONFIG` says `PRESET_GEL`, `PRESET_FLOODED`, `PRESET_AGM` or `CURVE_DISABLED`:**
the charger is ignoring your charge target. Fix it:

```
CHGCURVESEL=0
```

then `o` twice (off, on) to latch it, and reboot with `z` to confirm it now reads `CUSTOM_CURVE`.

**If you saw `RSTE enabled`:** switch the charger's AC off and on once — SYSTEM_CONFIG
changes only take effect at AC power-up. Then `z` again; it should now say
`auto-restart: already enabled`.

✅ **Pass:** second boot shows `already enabled` and `CUSTOM_CURVE CURVE_ENABLED`.

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
