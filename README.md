My take on amit-nz BMS, which is amit-nz's take on BobbyBleacher's take on Collin's BMS ;) 

Source projects: https://github.com/amit-nz/TeslaBMS-ESP32.git and https://github.com/BobbyBleacher/TeslaBMS and https://github.com/collin80/TeslaBMS

Arduino compatible project (ported to work on cheap & common ESP32 boards) to interface with the BMS child boards on Tesla Model S modules; possibly works for X modules as well.

Full credit to [BobbyBleacher](https://github.com/BobbyBleacher/) for doing the porting work.

The board I use is a ESP32-S3-WROOM-1 (N16R8) - https://github.com/amit-nz/TeslaBMS-ESP32/blob/master/esp32-s3-n16r8-development-board.png

Using this project, I monitor 2 x Tesla modules in a stationary ESS application.

Features:

- Balances to the lowest cell between two packs connected together in series.
- Cells balance for 30s and then update again, turning the balancers off momentarily to get accurate readings.
- Baud set 631578 (Use 612500 for older Tesla packs??)\
- Display on 4" LCD screen
- CSV output to FTP server.
- JSON endpoint to get module details.
- Push data into Home Assistant via MQTT (cell group voltage readings, module temperature (+) and (-) and min/max).
- WebUI for configuring various settings.
- Tested on the readily available [ESP32-S3-WROOM-1 N16R8 Development Board](https://github.com/amit-nz/TeslaBMS-ESP32/blob/master/esp32-s3-n16r8-development-board.png).

To-do:
- Refine LED flash states and put them in a readme!
- Simple WebUI that runs on the ESP32 for showing cell group/module states.
- Small OLED/E-Ink Display to show various data pertaining to the system.
- Ability to set things like Balancing Voltages etc w/o requring to connect using a phone/computer to the WebUI.
- Add SSL to MQTT to secure communication between the ESP32 and HASS.
- Add SSL to WebUI to secure communication between the ESP32 and clients.

Wiring:

The modules are daisy-chained together with a TTL interface, in a "ring" topology. 

The interface uses a Molex 15-97-5101 connector (but I just chopped off the end connector on the harness and used wago blocks).

Pinouts (original wiring harness): 
* Red = 5V / 3.3V input to the module (I use 3.3V out from the ESP32-S3-WROOM-1)
  * Note: this is just to signal the BMBs to wake up - they're powered internally from the modules themselves
* Green = Gnd for power and signal
* Gray = Fault output
* Yellow = UART Wire
* Blue = UART Wire

The fault output is active low. Use your own pull up to the fault line and if the line is pulled low then a fault has occurred.

Here is a PDF that explains how the wiring between modules and the master board is supposed to be:
https://cdn.hackaday.io/files/10098432032832/wiring.pdf

Disclaimer:
Use at your own risk! Working with LiIon batteries is dangerous.
