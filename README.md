# TeslaBMS-ESP32

An ESP32-S3 battery monitor and charger controller for **Tesla Model S battery modules**. It talks to the Tesla BMS boards (BMBs) on each module, balances the cells, controls a **MEAN WELL NPB-750-24** charger over CAN bus, and serves a full web interface from its own Wi-Fi access point, so it works with no home network or internet connection.

I use it to power a scissor lift, but nothing in it is specific to that. It should suit any electric vehicle or storage application where you want to charge and monitor Tesla modules.

> **Use at your own risk.** Working with lithium-ion batteries is dangerous. Read [Safety](#safety) before you connect anything.

## Credits

This is my take on amit-nz's BMS, which is amit-nz's take on BobbyBleacher's take on Collin's BMS. Full credit to the original authors:

- [collin80/TeslaBMS](https://github.com/collin80/TeslaBMS): the original Tesla BMB protocol work
- [BobbyBleacher/TeslaBMS](https://github.com/BobbyBleacher/TeslaBMS): the ESP32 port
- [amit-nz/TeslaBMS-ESP32](https://github.com/amit-nz/TeslaBMS-ESP32): the version this one was forked from

Compared with the version it was forked from, this one adds CAN charger control, a 4" LCD, a full on-device web UI, and a PlatformIO build. It drops the MQTT / Home Assistant and FTP export features.

## Features

- **Reads every cell** (voltage) and both module temperatures from each Tesla BMB, and finds and numbers the boards automatically.
- **Balances the cells.** Modules are handled in pairs, and each pair balances to the lowest cell in that pair.
- **Protects the pack.** Adjustable cell voltage and temperature limits, plus faults for a module that stops answering, fewer modules found than expected, and a failed temperature sensor. Any fault shows on the LCD, in the web UI, and on a buzzer.
- **Controls a MEAN WELL NPB-750-24 charger** over CAN bus:
  - an everyday "daily" charge target (about 80% charge) and a one-tap full-charge override,
  - the full charge curve (constant current, constant voltage, float, taper cutoff, restart voltage, stage timeouts),
  - a safety interlock that keeps the charger switched **off** for as long as any fault is active, and resumes charging automatically when the fault clears.
- **Its own Wi-Fi access point and web UI. No home Wi-Fi or internet needed.** The whole page is stored on the device. Tabs: Dashboard, Charging, Faults, Settings, Firmware update, and a live serial Console.
- **4" LCD** (480x320) showing cell voltages, temperatures, state of charge, faults and charging status. The LCD and the web dashboard show the first two modules; the BMS itself supervises every module it finds. When there is more than one page to show (dashboard, faults, charging), the LCD cycles through all of them, 8 seconds each.
- **Onboard LED and buzzer** for at-a-glance state.
- **Wireless firmware updates** from the web UI.
- **Optional home Wi-Fi** connection, in addition to the access point.
- **JSON endpoint** (`/api/state`) with the live state, for your own tools.

State of charge is an **estimate** based on average cell voltage. The device does not count current, so treat it as a rough guide.

## Hardware

| Part | Notes |
|---|---|
| ESP32-S3 board with 16MB flash / 8MB PSRAM | I use an ESP32-S3-WROOM-1 N16R8 dev board. It follows the layout of Espressif's [ESP32-S3-DevKitC-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html), which has the pinout diagram |
| Tesla Model S battery module(s) | The code is tuned for **6S** modules; my build is two modules in parallel |
| MEAN WELL **NPB-750-24** charger | With a 3.3V CAN transceiver, such as a Waveshare SN65HVD230 |
| 4" ST7796S SPI LCD, 480x320 | |
| Buzzer | |
| Custom PCB | See below |

### Custom PCB

I built a custom PCB to connect everything above. **The schematic is here: [docs/schematic.pdf](docs/schematic.pdf).** The PCB layout files are available on request; open an issue on this repository and ask. The schematic and the pin table below are also everything you need to wire your own version on a prototype board.

### ESP32-S3 pin assignments

| Function | GPIO |
|---|---|
| Tesla BMB serial RX / TX | 16 / 17 |
| Tesla BMB fault line (active low) | 4 |
| Charger CAN TX / RX | 2 / 1 |
| Buzzer | 21 |
| LCD SCLK / MOSI / DC / CS / RST / backlight | 11 / 12 / 13 / 14 / 9 / 10 |

Pins are defined at the top of [`src/main.cpp`](src/main.cpp), and the LCD pins in [`src/Displaymanager.h`](src/Displaymanager.h). If the picture is upside down for how you mounted the display, change `LCD_ROTATION` in that same file (it is currently `3`; `1` turns it 180 degrees).

### Wiring the Tesla modules

The modules are daisy-chained together with a TTL interface, in a ring topology. The interface uses a Molex 15-97-5101 connector (or chop the end off the harness and use Wago blocks).

Pinouts (original wiring harness):

- Red = 5V / 3.3V input to the module (I use 3.3V from the ESP32). This only wakes the BMBs; they are powered internally from the modules.
- Green = ground for power and signal
- Gray = fault output
- Yellow = UART wire
- Blue = UART wire

The fault output is active low. The firmware enables the ESP32's internal pull-up on GPIO 4, so no external pull-up is needed. If a BMB pulls the line low, the firmware reports a fault.

A PDF explaining how the wiring between the modules and the master board is supposed to work: <https://cdn.hackaday.io/files/10098432032832/wiring.pdf>

The serial speed is set by `BMS_BAUD` in `src/main.cpp` (default 631578; other values that may suit older Tesla packs are 612500, 617647 and 608695).

### Wiring the charger

The NPB-750-24 connects through its 14-pin control connector: pin 11 = CANH, pin 12 = CANL, pins 9 and 10 = ground. The CAN bus runs at 250 kbit/s. The charger's address pins (A0 / A1) select its bus address; the code assumes both are left unconnected, which is address 3.

**The charger must be in auto-ranging mode** for the voltage and current commands to take effect (see the MEAN WELL manual: all DIP switches off, do the ON to OFF sequence within 15 seconds, and jumper pins 7 and 8). MEAN WELL describes auto-ranging as being for lithium batteries that have a BMS.

## Building and flashing

You need [PlatformIO](https://platformio.org/). It works from VS Code, CLion (PlatformIO plugin) or the command line.

1. Clone this repository and open the folder.
2. In the `src/` folder, copy `secrets_template.h` to a new file named **`secrets.h`** and fill in your values. `secrets.h` is git-ignored so your details are never committed.
   - Set `SECRET_AP_SSID` to the name you want for the device's Wi-Fi network.
   - The MQTT and FTP entries are unused, but the code still expects them to exist, so leave the placeholder values.
3. Build and upload:

   ```
   pio run -t upload
   ```

4. Optionally watch the serial output (115200 baud):

   ```
   pio device monitor
   ```

The platform and every library are pinned in `platformio.ini` to the exact versions this firmware was tested with, so your build matches mine. A GitHub Actions job also builds the firmware on Linux for every change. `platformio.ini` is set up for the N16R8 board (16MB flash, octal PSRAM). If your board is different, change the board and memory settings there. The display draws into PSRAM buffers, so the PSRAM settings in that file matter: without them the Arduino core does not detect the PSRAM.

## Using it

1. Power the device and connect a phone or laptop to its Wi-Fi network (the name you set in `secrets.h`).
2. Open **`http://Lift.local`** or **`http://Lift.iot`**, or go straight to **`http://192.168.44.1`**.
   - The hostname (`Lift` by default) can be changed in the Settings tab.
   - `.local` works out of the box on iPhone and Mac. On Windows it needs Bonjour. `.iot` and the IP address work everywhere.

### Open by default: no Wi-Fi password, no login

As shipped, the Wi-Fi access point is **open** and the web UI has **no login**, so anyone within Wi-Fi range can change the charger and pack settings and upload new firmware. That is a deliberate convenience for a device that lives on a machine, but you should think about who could be near it.

Two switches in [`src/config.h`](src/config.h) control this. Set either to `1` to turn the protection back on, using the credentials from `secrets.h`:

```cpp
#define AP_REQUIRE_PASSWORD  0   // 1 = the Wi-Fi network needs a password
#define WEBUI_REQUIRE_AUTH   0   // 1 = the web UI asks for a username and password
```

**The web login is not enough on its own.** The Console tab does not use it, so anyone who can reach the device can still send it commands. If you want protection, turn on `AP_REQUIRE_PASSWORD` as well: the Wi-Fi password is what actually keeps people out.

### Number of modules

Set **Packs configured** in the Settings tab to the number of Tesla modules you have (default 2). The BMS raises a fault, and keeps the charger off, in these cases:

- fewer modules are found than you configured (`MODULES x/y FOUND`),
- a module has not answered for about 10 seconds (`NO COMMS`),
- a module's temperature sensor gives an impossible reading (`TEMP SENSOR`).

While modules are missing, the BMS keeps trying to find them. Each attempt resets the boards, so it tries after 10 seconds, then 20, 40, and then once a minute, going back to 10 seconds once they are all found.

### What the LED and buzzer mean

The LED is the RGB LED built into the ESP32-S3 dev board. The firmware drives it on GPIO 48 (`PIN` in `src/main.cpp`), which is where the original ESP32-S3-DevKitC-1 has it. Espressif's newer v1.1 revision uses GPIO 38 instead, so if your LED never lights, change `PIN` there.

- **Solid red:** starting up.
- **Solid yellow:** not connected to a home Wi-Fi network. This is normal if you only use the device's own access point.
- **Blue flashes:** connected to your home Wi-Fi and reading the modules.
- **Purple flashes:** fewer modules found than configured. It keeps searching.
- **Green flashes:** all configured modules found.
- **Buzzer, repeating double chirp:** a fault that blocks charging is active, or a low-voltage alarm is active while the charger is off.

### Charging

The charger is controlled from the Charging tab (or the serial console):

- **Daily target** is the everyday charge voltage (default 24.0V for a 6S pack, about 80%).
- **Full-charge override** charges to a higher target (default 24.9V) once, and returns to the daily target when the charger reports it is full.
- Curve settings, timeouts and the charger output switch are all on the same tab.

Each time the charger switches on, the firmware reads its charge settings (current, voltage, float, taper) back and writes to the log whether they match what was sent. By default a mismatch is only logged; set `CHARGER_VERIFY_TURNS_OFF` to `1` in `src/config.h` to also switch the charger off.

Whenever a fault that makes charging unsafe is active (over-voltage, over-temperature, a lost module, a sensor failure and so on), the charger output is forced off, and it is not possible to switch it on. **Low cell voltage is different:** a cell below the low-voltage limit sounds the alarm but does not stop charging, so a low pack can always be recharged. Only a cell below 2.5 V (shown as `DEEP DISCHARGE`) blocks charging. The buzzer for a low-voltage alarm is silent while the charger is running, and sounds again if charging stops while the pack is still low. When the fault clears, the charger returns to the state it was in before: if it was running it turns back on by itself, and if it was off it stays off. After a reboot or power cut the charger always starts off.

### Default limits

These are the first-boot defaults. Everything can be changed in the web UI Settings tab or from the serial console, and changes are saved on the device.

| Setting | Default |
|---|---|
| Cell over-voltage / under-voltage | 4.20V / 3.30V |
| Over-temperature / under-temperature | 65 °C / -10 °C |
| Balance starts at | 3.95V (hysteresis 0.007V) |
| Charger current | 22.5A (taper cutoff 2.25A) |

The charger defaults are for a **6S** pack. For any other pack size, change the charger voltages and the cell limits to suit your pack, and set `CELLS_IN_SERIES` in `src/config.h`.

Two hard limits protect against typos: the cell over-voltage limit cannot be set above 4.25 V, and every charger voltage setting is capped at `CELLS_IN_SERIES` times that limit (25.2 V for the default 6S pack), even though the charger itself can go higher. The driver also refuses values outside the NPB-750-24's own range. None of this can know what is safe for your particular battery, so check the numbers.

### Serial console

Connect over USB at 115200 baud, or use the Console tab in the web UI. Type `h` for the full menu. Some useful commands:

| Command | Action |
|---|---|
| `o` | Toggle charger output on/off |
| `y` | Print charger status |
| `u` | Toggle full-charge override |
| `t` | Inject a 5-second test fault (checks the buzzer, display and web UI) |
| `k` | Show each module's own over- and under-voltage trip points (read-only) |
| `x` | Pretend all modules stopped answering for 40 seconds (checks the `NO COMMS` fault and the charger cut-off) |
| `B` | Run a balancing pass now (`b` does the same) |
| `F`, `R`, `C` | Find boards, renumber boards, clear board faults |
| `VOLTLIMHI=4.2` | Set a cell voltage limit (there are matching commands for the other limits) |
| `CHGDAILYV=24.0` | Set the daily charge target |

## Safety

- Lithium-ion packs can catch fire or explode if mis-wired, over-charged, over-discharged or short-circuited. Only build this if you understand the risks.
- If the main loop stops running for 30 seconds (a software hang), the ESP32 reboots itself, and the charger output is switched off during boot. That is a last resort, not a substitute for the next point.
- **The software fault interlock only works while the ESP32 is running.** It is one layer of protection, not the only one. Use hardware protection as well, such as a fuse, a contactor or hardware cutoff, and consider gating the charger's remote on/off pins with a hardware safety circuit.
- The state of charge shown is an estimate, not a measurement.
- Check every charger and cell limit against your own pack before charging. The defaults are for one specific 6S setup.
- Firmware updates over Wi-Fi are not password protected by default (see [Open by default](#open-by-default-no-wi-fi-password-no-login)).
- Nothing here is certified for any purpose. You are responsible for what you build.

## License

Released under the [MIT License](LICENSE): you may use, change and share this code, including commercially, as long as the copyright notice and license text stay with it. It comes with no warranty.

Some files carry their original authors' copyright notices at the top (`Logger.h`, `Logger.cpp`, `SerialConsole.h`, `SerialConsole.cpp`, by Collin Kidder and others, also under the MIT License). Those notices must be kept. See [Credits](#credits) for the projects this one is built on.
