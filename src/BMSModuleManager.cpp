#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <FS.h>              // File System support
//#include <LittleFS.h>
#include <ESP32_FTPClient.h> // FTP Client library

extern EEPROMSettings settings;
extern int packsConfigured;   // how many modules the user says the pack has (main.cpp)
int numFoundModules = 0;
float lowestCellVolt;
float highestCellVolt;
float lowestPackTemp;
float highestPackTemp;


BMSModuleManager::BMSModuleManager(AsyncWebServer* webServer)
{
    for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
        modules[i].setExists(false);
        modules[i].setAddress(i);
    }
    lowestPackVolt = 1000.0f;
    highestPackVolt = 0.0f;
    lowestPackTemp = 200.0f;
    highestPackTemp = -100.0f;
    for (int i = 0; i < MAX_ACTIVE_FAULTS; i++) {
        faultList[i].active = false;
        faultList[i].id[0] = '\0';
        faultList[i].reason[0] = '\0';
        faultList[i].startMillis = 0;
    }
    memset(commFails, 0, sizeof(commFails));
    server = webServer;
}

// void BMSModuleManager::balanceCells()
// {
//   float lowestCell = 10.0f;
//   for (int x = 1; x <= MAX_MODULE_ADDR; x++) //start cycling thru packs
//     {
//         if (modules[x].isExisting()) //end the loop if we're out of packs
//         {
//             modules[x].readModuleValues(); //get some data
//             if (modules[x].getLowCellV() < lowestCell) lowestCell = modules[x].getLowCellV(); //Determine lowest value of of a pack and set lowestCell
//             // Serial.println("In balanceCells() " + String(lowestCell, 3) + " Module " + String(x));

//             if ( x%2 == 0) //Run this code if we're on an even numbered pack (so we do this for every two packs i.e, each string
//             {
//               modules[x-1].balanceCells(lowestCell); //Balance to the lowestCell (potentially) for the previously number pack. Tolerance is 0.007f
//               modules[x].balanceCells(lowestCell); //Balance to the lowestCell (potentially) for the current pack
//               // Serial.println("In balanceCells() 2 " + String(lowestCell, 3) + " Module " + String(x));
//               lowestCell = 10.0f; //Reset lowestCell value for the next two packs
//             }
//         }
//     }
// }
void BMSModuleManager::balanceCells(bool refreshReadings)
{
    float lowestCell = 10.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++) // Start cycling through packs
    {
        if (modules[x].isExisting()) // Process only if the module exists
        {
            if (refreshReadings) modules[x].readModuleValues(); // Get module data

            if (x % 2 != 0)  // If this is an odd-numbered pack, initialize lowestCell tracking
            {
                lowestCell = modules[x].getLowCellV();
            }
            else  // If this is an even-numbered pack, complete the pair
            {
                float secondLowestCell = modules[x].getLowCellV();
                lowestCell = min(lowestCell, secondLowestCell); // Get the lowest voltage of the pair

                modules[x - 1].balanceCells(lowestCell); // Balance previous pack
                modules[x].balanceCells(lowestCell);     // Balance current pack

                lowestCell = 10.0f; // Reset for the next pair
            }
        }
    }
}


void BMSModuleManager::balanceCell(int cellNumber)
{
    for (int address = 1; address <= MAX_MODULE_ADDR; address++)
    {
        if (modules[address].isExisting()) modules[address].balanceCell(cellNumber);
    }
}

/*
 * Try to set up any uninitialized boards. Send a command to address 0 and see if there is a response. If there is then there is
 * still at least one uninitialized board. Go ahead and give it the first ID not registered as already taken.
 * If we send a command to address 0 and no one responds then every board is inialized and this routine stops.
 * Don't run this routine until after the boards have already been enumerated.
 * Note: The 0x80 conversion it is looking might in theory block the message from being forwarded so it might be required
 * To do all of this differently. Try with multiple boards. The alternative method would be to try to set the next unused
 * address and see if any boards respond back saying that they set the address.
 */
void BMSModuleManager::setupBoards()
{
    uint8_t payload[3];
    uint8_t buff[10];
    int retLen;

    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;

    // Bounded: one pass per possible address, plus a few spare. With no cap, a
    // board that answers at address 0 but never accepts its new address made
    // this loop run forever and stall everything else, including the charger
    // interlock.
    int guard = 0;
    while (guard++ < 70)
    {
        payload[0] = 0;
        payload[1] = 0;
        payload[2] = 1;
        retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
        if (retLen == 4)
        {
            if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1)
            {
                Logger::debug("00 found");
                //look for a free address to use
                for (int y = 1; y < 63; y++)
                {
                    if (!modules[y].isExisting())
                    {
                        payload[0] = 0;
                        payload[1] = REG_ADDR_CTRL;
                        payload[2] = y | 0x80;
                        BMSUtil::sendData(payload, 3, true);
                        delay(3);
                        if (BMSUtil::getReply(buff, 10) > 2)
                        {
                            if (buff[0] == (0x81) && buff[1] == REG_ADDR_CTRL && buff[2] == (y + 0x80))
                            {
                                modules[y].setExists(true);
                                numFoundModules++;
                                Logger::debug("Address assigned");
                            }
                        }
                        break; //quit the for loop
                    }
                }
            }
            else
            {
              Logger::debug("nobody responded properly to the zero address so our work here is done.");
              break; //nobody responded properly to the zero address so our work here is done.
            }
        }
        else break;
    }
    if (guard > 70) Logger::error("setupBoards: gave up after 70 passes -- a board keeps answering at address 0");
}

void BMSModuleManager::findBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];

    numFoundModules = 0;
    payload[0] = 0;
    payload[1] = 0; // read registers starting at 0
    payload[2] = 1; // read one byte

    int lastGood = 0;

    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        modules[x].setExists(false);
        payload[0] = x << 1;

        Logger::debug("Probing module address: %X", x);
        BMSUtil::sendData(payload, 3, false);
        delay(20);

        if (BMSUtil::getReply(buff, 8) > 4)
        {
            if (buff[0] == (x << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0)
            {
                modules[x].setExists(true);
                numFoundModules++;
                lastGood = x;
                Logger::debug("Found module with address: %X", x);
            }
            else
            {
                Logger::debug("Received malformed or incorrect reply from module %X", x);
                break;
            }
        }
        else
        {
            Logger::debug("No response from module address: %X", x);
            break; // Stop searching — likely break in chain
        }

        delay(5);
    }

    if (lastGood > 0)
    {
        Logger::info("Last responding module: %X", lastGood);
    }
    else
    {
        Logger::warn("No modules responded to findBoards()");
    }
}


/*
 * Force all modules to reset back to address 0 then set them all up in order so that the first module
 * in line from the master board is 1, the second one 2, and so on.
*/
void BMSModuleManager::renumberBoardIDs()
{
    uint8_t payload[3];
    uint8_t buff[8];
    int attempts = 1;

    for (int y = 1; y < 63; y++)
    {
        modules[y].setExists(false);
        numFoundModules = 0;
    }

    while (attempts < 3)
    {
        payload[0] = 0x3F << 1; //broadcast the reset command
        payload[1] = 0x3C;//reset
        payload[2] = 0xA5;//data to cause a reset
        BMSUtil::sendData(payload, 3, true);
        delay(100);
        BMSUtil::getReply(buff, 8);
        if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57) break;
        attempts++;
    }

    setupBoards();
}

/*
After a RESET boards have their faults written due to the hard restart or first time power up, this clears thier faults
*/
void BMSModuleManager::clearFaults()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Alert Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[1] = REG_FAULT_STATUS;//Fault Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    // Note: this only clears the BMB's own hardware fault registers. Any
    // software/voltage-limit-based faults in the registry will simply
    // re-trigger on the next poll if the underlying condition (e.g. a cell
    // still over VOLTLIMHI) hasn't actually gone away -- which is correct.
}

/*
Puts all boards on the bus into a Sleep state, very good to use when the vehicle is a rest state.
Pulling the boards out of sleep only to check voltage decay and temperature when the contactors are open.
*/

void BMSModuleManager::sleepBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x04;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

/*
Wakes all the boards up and clears thier SLEEP state bit in the Alert Status Registery
*/

void BMSModuleManager::wakeBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x00;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);

    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Fault Status
    payload[2] = 0x04;//data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

// void BMSModuleManager::getAllVoltTemp()
// {
//     packVolt = 0.0f;
//     lowestCellVolt = 4.3f;
//     highestCellVolt = 0.0f;
//     lowestPackTemp = 50.0f;
//     highestPackTemp = -10.0f;
//     for (int x = 1; x <= MAX_MODULE_ADDR; x++)
//     {
//     if (modules[x].isExisting())
//     {
//       modules[x].stopBalance();
//     }
//   }
//   delay(1000);
//     for (int x = 1; x <= MAX_MODULE_ADDR; x++)
//     {
//         if (modules[x].isExisting())
//         {
//             Logger::debug("");
//             Logger::debug("Module %i exists. Reading voltage and temperature values", x);
//             modules[x].readModuleValues();
//             Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
//             Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
//             Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
//             packVolt += modules[x].getModuleVoltage();
//             if (modules[x].getLowCellV() < lowestCellVolt) lowestCellVolt = modules[x].getLowCellV();
//             if (modules[x].getHighCellV() > highestCellVolt) highestCellVolt = modules[x].getHighCellV();
//             if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
//             if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();
//         }
//     }
//     // Debug summary
//     //Serial.println("");
//     //Serial.println("High temp: " + String(highestPackTemp));
//     //Serial.println("Low temp: " + String(lowestPackTemp));
//     //Serial.println("High volt: " + String(highestCellVolt));
//     //Serial.println("Low volt: " + String(lowestCellVolt));

//     if (packVolt > highestPackVolt) highestPackVolt = packVolt;
//     if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;
// //You can uncomment this code if you do have the module fault chain attached. Change the pin number to where it is attached
// /*
//     if (digitalRead(13) == LOW) {
//         if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
//         isFaulted = true;
//     }
//     else
//     {
//         if (isFaulted) Logger::info("All modules have exited a faulted state");
//         isFaulted = false;
//     }
// */
// }

void BMSModuleManager::getAllVoltTemp()
{
    packVolt = 0.0f;
    lowestCellVolt = 4.3f;
    highestCellVolt = 0.0f;
    lowestPackTemp = 50.0f;
    highestPackTemp = -10.0f;

    // Counts modules that produced a successful read this cycle. Used below
    // to AVERAGE packVolt across modules rather than summing them -- these
    // two modules are wired in PARALLEL (shared bus), not series, so the
    // pack voltage is the single shared bus voltage (~20V), not ~41V. If a
    // third module is ever added, this still does the right thing as long
    // as all modules stay on the same parallel bus.
    int moduleReadCount = 0;

    // Reset software fault state each cycle -- it gets re-set below if any
    // module reports a fault via its registers. This is the BMB-register
    // side of fault detection; the hardware FAULT line (GPIO4) is checked
    // separately in main.cpp's loop() and ORed in when building DisplayData.
    // First, stop balancing on all existing modules
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            modules[x].stopBalance();
        }
    }

    delay(1000);

    // Then, attempt to read all modules' values
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            Logger::debug("");
            Logger::debug("Module %i exists. Reading voltage and temperature values", x);

            // Try reading the module values, retrying if necessary
            if (modules[x].readModuleValues()) {
                // Only process the module if the data was successfully read
                commFails[x] = 0;
                Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
                Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
                Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));

                packVolt += modules[x].getModuleVoltage();
                moduleReadCount++;

                // Update the lowest and highest values
                if (modules[x].getLowCellV() < lowestCellVolt) lowestCellVolt = modules[x].getLowCellV();
                if (modules[x].getHighCellV() > highestCellVolt) highestCellVolt = modules[x].getHighCellV();
                if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
                if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();

                // Enforce the configurable voltage/temperature limits
                // (VOLTLIMHI, VOLTLIMLO, TEMPLIMHI, TEMPLIMLO). These were
                // previously stored in `settings` but never actually checked
                // against a live reading -- this is that check. Only run
                // against data from a successful read (the block we're in)
                // so a failed/zeroed read can't falsely look like an
                // under-voltage or under-temperature condition.
                //
                // Each condition gets a stable id (e.g. "M1C4OV") so it's
                // recognized as the SAME ongoing fault across polls rather
                // than a brand new one every 3 seconds -- that's what lets
                // the fault page show "how long has this been happening".
                for (int c = 0; c < 6; c++) {
                    float v = modules[x].getCellVoltage(c);
                    char idOV[12], idUV[12], reason[40];

                    snprintf(idOV, sizeof(idOV), "M%dC%dOV", x, c + 1);
                    if (v > settings.OverVSetpoint) {
                        snprintf(reason, sizeof(reason), "MOD%d C%d OVER-V %.2fV", x, c + 1, v);
                        reportFault(idOV, reason);
                        Logger::error("%s", reason);
                    } else {
                        clearFaultById(idOV);
                    }

                    snprintf(idUV, sizeof(idUV), "M%dC%dUV", x, c + 1);
                    if (v < settings.UnderVSetpoint) {
                        snprintf(reason, sizeof(reason), "MOD%d C%d UNDER-V %.2fV", x, c + 1, v);
                        reportFault(idUV, reason);
                        Logger::error("%s", reason);
                    } else {
                        clearFaultById(idUV);
                    }
                }

                float modLowTemp  = modules[x].getLowTemp();
                float modHighTemp = modules[x].getHighTemp();
                char idOT[12], idUT[12], tReason[40];

                snprintf(idOT, sizeof(idOT), "M%dOT", x);
                if (modHighTemp > settings.OverTSetpoint) {
                    snprintf(tReason, sizeof(tReason), "MOD%d OVER-TEMP %.1fC", x, modHighTemp);
                    reportFault(idOT, tReason);
                    Logger::error("%s", tReason);
                } else {
                    clearFaultById(idOT);
                }

                snprintf(idUT, sizeof(idUT), "M%dUT", x);
                if (modLowTemp < settings.UnderTSetpoint) {
                    snprintf(tReason, sizeof(tReason), "MOD%d UNDER-TEMP %.1fC", x, modLowTemp);
                    reportFault(idUT, tReason);
                    Logger::error("%s", tReason);
                } else {
                    clearFaultById(idUT);
                }

                // A shorted or open thermistor gives NaN, which would slip past
                // both temperature checks above. Report it as its own fault.
                char idSens[12], sReason[40];
                snprintf(idSens, sizeof(idSens), "M%dSENS", x);
                if (!modules[x].hasValidTemperatures()) {
                    snprintf(sReason, sizeof(sReason), "MOD%d TEMP SENSOR", x);
                    reportFault(idSens, sReason);
                    Logger::error("%s", sReason);
                } else {
                    clearFaultById(idSens);
                }
            }
            else {
                // Log failure to read module
                Logger::error("Failed to read module %i. Skipping module...", x);
                if (commFails[x] < 255) commFails[x]++;
            }

            // A module that keeps failing to answer can't be supervised: its
            // cell values above are stale. After BMB_COMM_FAIL_LIMIT failed
            // cycles in a row, raise a fault so the charger interlock trips
            // instead of charging on old readings. Clears on the next good read.
            char idComm[12], cReason[40];
            snprintf(idComm, sizeof(idComm), "M%dCOMM", x);
            if (commFails[x] >= BMB_COMM_FAIL_LIMIT) {
                snprintf(cReason, sizeof(cReason), "MOD%d NO COMMS", x);
                reportFault(idComm, cReason);
            } else {
                clearFaultById(idComm);
            }

            // Check the module's fault register regardless of whether the
            // full voltage/temp read above succeeded -- readStatus() (called
            // at the top of readModuleValues()) populates faults/alerts on
            // every poll independent of the CRC-checked ADC data block.
            char idReg[12];
            snprintf(idReg, sizeof(idReg), "M%dREG", x);
            if (modules[x].getFaults() != 0) {
                uint8_t f = modules[x].getFaults();
                char regReason[40];
                if (f & 1)         snprintf(regReason, sizeof(regReason), "MOD%d OVER-VOLT (REG)", x);
                else if (f & 2)    snprintf(regReason, sizeof(regReason), "MOD%d UNDER-VOLT (REG)", x);
                else if (f & 4)    snprintf(regReason, sizeof(regReason), "MOD%d CRC ERROR", x);
                else if (f & 8)    snprintf(regReason, sizeof(regReason), "MOD%d POWER-ON-RESET", x);
                else if (f & 0x10) snprintf(regReason, sizeof(regReason), "MOD%d TEST FAULT (REG)", x);
                else if (f & 0x20) snprintf(regReason, sizeof(regReason), "MOD%d REG INCONSISTENT", x);
                else               snprintf(regReason, sizeof(regReason), "MOD%d FAULT REG 0x%02X", x, f);
                reportFault(idReg, regReason);
            } else {
                clearFaultById(idReg);
            }
        }
    }

    // Modules are wired in PARALLEL, not series -- average the readings
    // instead of summing them so packVolt reflects the shared bus voltage
    // (~20V) rather than a fictitious series total (~41V).
    if (moduleReadCount > 0) packVolt /= moduleReadCount;

    // Fewer modules than the user says the pack has means some cells are not
    // being monitored (or the BMB ring is broken), so hold the charger off.
    // (Extra modules beyond the configured count are still monitored, so they
    // don't raise this.)
    if (numFoundModules < packsConfigured) {
        char mReason[40];
        snprintf(mReason, sizeof(mReason), "MODULES %d/%d FOUND", numFoundModules, packsConfigured);
        reportFault("MODS", mReason);
    } else {
        clearFaultById("MODS");
    }

    // Faults are only cleared for modules that still exist (above), so a fault
    // belonging to a module that has since dropped out of the ring would stay
    // stuck on forever. Per-module fault ids all start with "M<number>"; drop
    // any whose module no longer exists. (The missing module itself is what
    // the MODS fault above reports.)
    for (int x = 1; x <= MAX_MODULE_ADDR; x++) {
        if (!modules[x].isExisting()) commFails[x] = 0;
    }
    for (int i = 0; i < MAX_ACTIVE_FAULTS; i++) {
        if (faultList[i].active && faultList[i].id[0] == 'M' && isdigit((unsigned char)faultList[i].id[1])) {
            int m = atoi(&faultList[i].id[1]);
            if (m >= 1 && m <= MAX_MODULE_ADDR && !modules[m].isExisting()) {
                faultList[i].active = false;
            }
        }
    }

    // Update the overall pack voltage bounds
    if (packVolt > highestPackVolt) highestPackVolt = packVolt;
    if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

    // Optionally handle fault state, uncomment if needed
    /*
    if (digitalRead(13) == LOW) {
        if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
        isFaulted = true;
    }
    else
    {
        if (isFaulted) Logger::info("All modules have exited a faulted state");
        isFaulted = false;
    }
    */
}


float BMSModuleManager::getPackVoltage()
{
    return packVolt;
}

float BMSModuleManager::getAvgTemperature()
{
    if (numFoundModules <= 0) return 0.0f;   // avoid 0/0 = NaN when no modules are found
    float avg = 0.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) avg += modules[x].getAvgTemp();
    }
    avg = avg / (float)numFoundModules;

    return avg;
}

float BMSModuleManager::getAvgCellVolt()
{
    if (numFoundModules <= 0) return 0.0f;   // avoid 0/0 = NaN when no modules are found
    float avg = 0.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) avg += modules[x].getAverageV();
    }
    avg = avg / (float)numFoundModules;

    return avg;
}

void BMSModuleManager::printPackSummary()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;

    Logger::console("");
    Logger::console("");
    Logger::console("");
    Logger::console("                                     Pack Status:");
    if (getActiveFaultCount() > 0) Logger::console("                                       FAULTED!");
    else Logger::console("                                   All systems go!");
    Logger::console("Modules: %i    System Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules,
                    getPackVoltage(),getAvgCellVolt(), getAvgTemperature());
    Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Logger::console("                               Module #%i", y);

            Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                            modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());

            Serial.print("  Currently balancing cells: ");
            for (int i = 0; i < 6; i++)
            {
                if (modules[y].getBalancingState(i) == 1)
                {
                    Serial.print(i);
                    Serial.print(" ");
                }
            }
            Serial.println();

            if (faults > 0)
            {
                Logger::console("  MODULE IS FAULTED:");
                if (faults & 1)
                {
                    Serial.print("    Overvoltage Cell Numbers (1-6): ");
                    for (int i = 0; i < 6; i++)
                    {
                        if (COV & (1 << i))
                        {
                            Serial.print(i+1);
                            Serial.print(" ");
                        }
                    }
                    Serial.println();
                }
                if (faults & 2)
                {
                    Serial.print("    Undervoltage Cell Numbers (1-6): ");
                    for (int i = 0; i < 6; i++)
                    {
                        if (CUV & (1 << i))
                        {
                            Serial.print(i+1);
                            Serial.print(" ");
                        }
                    }
                    Serial.println();
                }
                if (faults & 4)
                {
                    Logger::console("    CRC error in received packet");
                }
                if (faults & 8)
                {
                    Logger::console("    Power on reset has occurred");
                }
                if (faults & 0x10)
                {
                    Logger::console("    Test fault active");
                }
                if (faults & 0x20)
                {
                    Logger::console("    Internal registers inconsistent");
                }
            }
            if (alerts > 0)
            {
                Logger::console("  MODULE HAS ALERTS:");
                if (alerts & 1)
                {
                    Logger::console("    Over temperature on TS1");
                }
                if (alerts & 2)
                {
                    Logger::console("    Over temperature on TS2");
                }
                if (alerts & 4)
                {
                    Logger::console("    Sleep mode active");
                }
                if (alerts & 8)
                {
                    Logger::console("    Thermal shutdown active");
                }
                if (alerts & 0x10)
                {
                    Logger::console("    Test Alert");
                }
                if (alerts & 0x20)
                {
                    Logger::console("    OTP EPROM Uncorrectable Error");
                }
                if (alerts & 0x40)
                {
                    Logger::console("    GROUP3 Regs Invalid");
                }
                if (alerts & 0x80)
                {
                    Logger::console("    Address not registered");
                }
            }
            if (faults > 0 || alerts > 0) Serial.println();
        }
    }
}
/*
void BMSModuleManager::printPackDetails()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;
    int cellNum = 0;
    Serial.println("");

    //Logger::console("");
    //Logger::console("");
    //Logger::console("");
    //Logger::console("                                         Pack Status:");
    //if (isFaulted) Logger::console("                                           FAULTED!");
    //else Logger::console("                                      All systems go!");
    //Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules,
    //                getPackVoltage(),getAvgCellVolt(), getAvgTemperature());
    //Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Serial.print("Module #");
            Serial.print(y);
            if (y < 10) Serial.print(" ");
            //Serial.print("  ");
            //Serial.print(modules[y].getModuleVoltage());
            //Serial.print("V");
            for (int i = 0; i < 6; i++)
            {
                if (cellNum < 10) Serial.print(" ");
                Serial.print("  Cell");
                if (cellNum < 10){Serial.print("0");}
                Serial.print(cellNum++ + 1);
                Serial.print(": ");
                Serial.print(modules[y].getCellVoltage(i), 3);
                Serial.print("V");
                if (modules[y].getBalancingState(i) == 1) Serial.print("*");
                else Serial.print(" ");
            }
            Serial.print("  Neg Term Temp: ");
            Serial.print(modules[y].getTemperature(0));
            Serial.print("C  Pos Term Temp: ");
            Serial.print(modules[y].getTemperature(1));
            Serial.print("C");
            if(isFaulted) Serial.println(" FAULTED!");
            else Serial.println("");
        }
    }
}
*/
void BMSModuleManager::printPackDetails()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;
    int cellNum = 0;
    Serial.println("");

    //Logger::console("");
    //Logger::console("");
    //Logger::console("");
    //Logger::console("                                         Pack Status:");
    //if (isFaulted) Logger::console("                                           FAULTED!");
    //else Logger::console("                                      All systems go!");
    //Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules,
    //                getPackVoltage(),getAvgCellVolt(), getAvgTemperature());
    //Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Serial.printf("Module #%02d  ", y);

            for (int i = 0; i < 6; i++)
            {
                Serial.printf("  Cell%03d: %.3fV", cellNum + 1, modules[y].getCellVoltage(i));
                if (modules[y].getBalancingState(i) == 1)
                    Serial.print("*");
                else
                    Serial.print(" ");
                cellNum++;
            }

            Serial.printf("  Neg Term Temp: %.2fC  Pos Term Temp: %.2fC",
                          modules[y].getTemperature(0), modules[y].getTemperature(1));
            Serial.println("");
        }
    }
}
void BMSModuleManager::printJsonData()
{
    for (int y = 1; y < 63; y++) {
        if (modules[y].isExisting()) {
            String msg = String(y);
            msg += ",";
            for (int i = 0; i < 6; i++) {
                msg += String(modules[y].getCellVoltage(i), 3);
                if (modules[y].getBalancingState(i) == 1) msg += "*";
                // Serial.print(modules[y].getBalancingState(i));
                msg += ",";
            }
            msg += modules[y].getTemperature(0);
            msg += ",";
            msg += modules[y].getTemperature(1);

            Serial.println(msg);
        }
    }
}
String BMSModuleManager::buildJsonData()
{
    String jsonResponse = "{\"packs\":[";
    bool firstModule = true;

    for (int y = 1; y < 63; y++) {
        if (modules[y].isExisting()) {
            if (!firstModule) {
                jsonResponse += ",";
            }
            firstModule = false;
            jsonResponse += "{";
            jsonResponse += "\"+\":\"" + String(modules[y].getTemperature(0), 2) + "\",";
            jsonResponse += "\"-\":\"" + String(modules[y].getTemperature(1), 2) + "\",";

            for (int i = 0; i < 6; i++) { // Loop through each cell in the module
            jsonResponse += "\"c" + String(i + 1) + "\":\"";
            jsonResponse += String(modules[y].getCellVoltage(i), 3); // Append cell voltage

            if (modules[y].getBalancingState(i) == 1) { // Check if cell is balancing
                jsonResponse += "*"; // Append asterisk if balancing
            }

            jsonResponse += "\""; // Close the value

            if (i < 5) jsonResponse += ","; // Add a comma except for the last cell
        }
            jsonResponse += ",\"module\":\"" + String(y) + "\"";
            jsonResponse += "}";
        }
    }
    jsonResponse += "]}";

    return jsonResponse;
}

void BMSModuleManager::handleBatteryStats(AsyncWebServerRequest* request, const String& bmsJson) {
    request->send(200, "application/json", bmsJson);
}

void BMSModuleManager::broadcastBatteryStats(AsyncWebSocket* ws, const String& bmsJson){
    ws->textAll(bmsJson);
    //Serial.println("Broadcasted WS JSON");
}

void BMSModuleManager::sendBatteryStats(String systemName, String ftpServer, String ftpUser, String ftpPassword, const String& bmsJson) {
    String systemNameLower = systemName;
    systemNameLower.toLowerCase();

    // Convert to C-style strings
    char ftpServerChar[64];
    char ftpUserChar[64];
    char ftpPasswordChar[64];

    ftpServer.toCharArray(ftpServerChar, sizeof(ftpServerChar));
    ftpUser.toCharArray(ftpUserChar, sizeof(ftpUserChar));
    ftpPassword.toCharArray(ftpPasswordChar, sizeof(ftpPasswordChar));

    // File names
    String tempFilename = systemNameLower + "_batterystats.json.tmp";
    String finalFilename = systemNameLower + "_batterystats.json";
    char tempChar[64];
    char finalChar[64];
    tempFilename.toCharArray(tempChar, sizeof(tempChar));
    finalFilename.toCharArray(finalChar, sizeof(finalChar));

    // This is the path on the ftp server where this system will drop off the  _batterystats.json file. Originally /tmp.
    String path = ".";

    ESP32_FTPClient* ftp = nullptr;  // Declare ftp pointer here so catch can see it

    try {
        ftp = new ESP32_FTPClient(ftpServerChar, ftpUserChar, ftpPasswordChar, 5000);

        ftp->OpenConnection();

        if (!ftp->isConnected()) {
            Serial.println("FTP error: could not connect to server");
            delete ftp;
            ftp = nullptr;
            return;
        }


        ftp->ChangeWorkDir(path.c_str());
        ftp->InitFile("Type I");

        ftp->NewFile(tempChar);
        ftp->WriteData((uint8_t*)bmsJson.c_str(), bmsJson.length());
        ftp->CloseFile();

        ftp->RenameFile(tempChar, finalChar);
//        Serial.println("FTP upload success: " + finalFilename);

        ftp->CloseConnection();
        delete ftp;
        ftp = nullptr;

    } catch (...) {
        Serial.println("Unknown FTP error occurred");
        if (ftp) {
            ftp->CloseConnection();
            delete ftp;
            ftp = nullptr;
        }
    }
}

void BMSModuleManager::publishIndividualData(PubSubClient& client, const char* baseTopic, String systemName) {
    for (int y = 1; y < 63; y++) {
        if (!modules[y].isExisting()) {
            continue;
        }
        // Publish cell voltages
        for (int i = 0; i < 6; i++) {
            String cellID = "p" + String(y) + "c" + String(i + 1);
            String cellValue = String(modules[y].getCellVoltage(i), 3);
            if(modules[y].getCellVoltage(i) > 2.5 && modules[y].getCellVoltage(i) < 4.29) {
              publishSensorData(client, baseTopic, systemName, cellID, "voltage", "V", 3, cellValue);
            }

        }
        // Publish temperatures
        const String terminalIDs[] = {"_neg", "_pos"};
        for (int t = 0; t < 2; t++) {
            String cellID = "p" + String(y) + terminalIDs[t];
            String cellValue = String(modules[y].getTemperature(t), 2);
            if(modules[y].getTemperature(t) > -10 && modules[y].getTemperature(t) < 50) {
              publishSensorData(client, baseTopic, systemName, cellID, "temperature", "°C", 2, cellValue);
            }

        }
    }

    // Publish system-wide max/min voltages and temperatures
    const String cellMetrics[] = {"max_value", "min_value"};
    const float cellValues[] = {highestCellVolt, lowestCellVolt};
    for (int i = 0; i < 2; i++) {
        if(cellValues[i] > 2.5 && cellValues[i] < 4.29){
          publishSensorData(client, baseTopic, systemName, cellMetrics[i], "voltage", "V", 3, String(cellValues[i], 3));
        }

    }

    const String tempMetrics[] = {"max_temp", "min_temp"};
    const float tempValues[] = {highestPackTemp, lowestPackTemp};
    for (int i = 0; i < 2; i++) {
      if(tempValues[i] > -10 && tempValues[i] < 50) {
        publishSensorData(client, baseTopic, systemName, tempMetrics[i], "temperature", "°C", 2, String(tempValues[i]));
      }

    }
    //Serial.println("Published data to Home Assistant.");

    // Debug summary
    //Serial.println("");
    //Serial.println("High temp: " + String(highestPackTemp));
    //Serial.println("Low temp: " + String(lowestPackTemp));
    //Serial.println("High volt: " + String(highestCellVolt));
    //Serial.println("Low volt: " + String(lowestCellVolt));
}

void BMSModuleManager::publishSensorData(PubSubClient& client, const char* baseTopic, String systemName, const String& cellID, const String& devClass, const String& unit, const int precision, const String& value) {
    String cellIDUpper = cellID;
    String systemNameLower = systemName;
    cellIDUpper.toUpperCase();
    systemNameLower.toLowerCase();

    String msg = "{\"name\":\"" + cellIDUpper + "\",\"dev_cla\":\"" + devClass + "\",\"unit_of_meas\":\"" + unit + "\",\"suggested_display_precision\":\"" + precision + "\",\"stat_t\":\"bms/sensor/" + systemNameLower + "_" +
             cellID + "/state\",\"uniq_id\":\"" + systemNameLower + "_" + cellID + "\",\"dev\":{\"ids\":[\"" + systemNameLower + "_bms\"],\"name\":\"" + systemName + "\", \"mf\":\"Bobby Martin\"}}";

    String config = baseTopic + systemNameLower + "_" + cellID + "/config";
    String state = "bms/sensor/" + systemNameLower + "_" + cellID + "/state";

    // Publish data
    client.publish(config.c_str(), msg.c_str(), true);
    client.publish(state.c_str(), value.c_str());

    // Debug logs
    //Serial.println("");
    //Serial.println("Published Sensor: " + cellID);
    //Serial.println(config + " | " + msg);
    //Serial.println(state + " | " + value);
}

// ── Display data builder ──────────────────────────────────────────────────────
// Fills a DisplayData struct from the current module state so the DisplayManager
// can render it.  Call this after getAllVoltTemp() has completed.
void BMSModuleManager::buildDisplayData(DisplayData& out) {
    // Pack-level scalars
    out.packVoltage    = packVolt;
    out.cellLow        = lowestCellVolt;
    out.cellHigh       = highestCellVolt;
    out.avgTemp        = getAvgTemperature();
    out.socPercent     = estimateSoC(getAvgCellVolt());
    out.uptimeSeconds  = millis() / 1000;
    out.numModules     = numFoundModules;

    // isFaulted and the fault list are derived live from the registry (not
    // a cached bool) so this reflects the current state of ALL fault
    // sources -- including ones reported from outside this class, like the
    // hardware FAULT pin and the serial console's test-fault injection --
    // even though this function itself only gets called from getAllVoltTemp()
    // every 3 seconds while those other sources get reported every second.
    out.activeFaultCount = getActiveFaultCount();
    out.isFaulted         = (out.activeFaultCount > 0);
    int faultSlot = 0;
    uint32_t nowMs = millis();
    for (int i = 0; i < MAX_ACTIVE_FAULTS && faultSlot < MAX_DISPLAY_FAULTS; i++) {
        if (faultList[i].active) {
            strncpy(out.faultReasons[faultSlot], faultList[i].reason, sizeof(out.faultReasons[faultSlot]) - 1);
            out.faultReasons[faultSlot][sizeof(out.faultReasons[faultSlot]) - 1] = '\0';
            out.faultDurationsMs[faultSlot] = nowMs - faultList[i].startMillis;
            faultSlot++;
        }
    }

    // Accumulate good/bad packet counts and balancing cell count across all modules
    out.goodPackets    = 0;
    out.badPackets     = 0;
    out.balancingCount = 0;

    // Per-module data – we map module index 1 → slot 0, module index 2 → slot 1.
    // If fewer than 2 modules are present, the unused slot is zeroed.
    for (int slot = 0; slot < 2; slot++) {
        out.moduleVolt[slot] = 0.0f;
        out.tempNeg[slot]    = 0.0f;
        out.tempPos[slot]    = 0.0f;
        for (int c = 0; c < 6; c++) {
            out.cellVolt[slot][c]      = 0.0f;
            out.cellBalancing[slot][c] = false;
        }
    }

    int slot = 0;
    for (int x = 1; x <= MAX_MODULE_ADDR && slot < 2; x++) {
        if (!modules[x].isExisting()) continue;

        out.moduleVolt[slot] = modules[x].getModuleVoltage();
        out.tempNeg[slot]    = modules[x].getTemperature(0);
        out.tempPos[slot]    = modules[x].getTemperature(1);

        for (int c = 0; c < 6; c++) {
            out.cellVolt[slot][c]      = modules[x].getCellVoltage(c);
            bool bal = (modules[x].getBalancingState(c) == 1);
            out.cellBalancing[slot][c] = bal;
            if (bal) out.balancingCount++;
        }

        // Packet stats – BMSModule doesn't expose these publicly yet, so we
        // leave out.goodPackets / badPackets for now (they stay 0).
        // When you add getGoodPackets()/getBadPackets() to BMSModule.h these
        // two lines replace the comment:
        //   out.goodPackets += modules[x].getGoodPackets();
        //   out.badPackets  += modules[x].getBadPackets();

        slot++;
    }
}

// ── Fault registry ────────────────────────────────────────────────────────────
// reportFault(): marks the fault identified by `id` as active. If it's
// already active, only the reason text is refreshed (e.g. an updated
// voltage reading) -- the original startMillis is preserved so duration
// tracking is continuous across polls rather than resetting every 3 seconds.
void BMSModuleManager::reportFault(const char* id, const char* reason) {
    for (int i = 0; i < MAX_ACTIVE_FAULTS; i++) {
        if (faultList[i].active && strcmp(faultList[i].id, id) == 0) {
            strncpy(faultList[i].reason, reason, sizeof(faultList[i].reason) - 1);
            faultList[i].reason[sizeof(faultList[i].reason) - 1] = '\0';
            return; // already active -- keep the original startMillis
        }
    }
    // Not currently active -- claim the first free slot as a brand new fault.
    for (int i = 0; i < MAX_ACTIVE_FAULTS; i++) {
        if (!faultList[i].active) {
            faultList[i].active = true;
            strncpy(faultList[i].id, id, sizeof(faultList[i].id) - 1);
            faultList[i].id[sizeof(faultList[i].id) - 1] = '\0';
            strncpy(faultList[i].reason, reason, sizeof(faultList[i].reason) - 1);
            faultList[i].reason[sizeof(faultList[i].reason) - 1] = '\0';
            faultList[i].startMillis = millis();
            return;
        }
    }
    // No free slots (shouldn't happen for a 2-module pack with 16 slots) --
    // silently drop rather than overflow; the condition will still show up
    // via the serial Logger::error() calls at the point it was detected.
}

void BMSModuleManager::clearFaultById(const char* id) {
    for (int i = 0; i < MAX_ACTIVE_FAULTS; i++) {
        if (faultList[i].active && strcmp(faultList[i].id, id) == 0) {
            faultList[i].active = false;
            return;
        }
    }
}

int BMSModuleManager::getActiveFaultCount() {
    int count = 0;
    for (int i = 0; i < MAX_ACTIVE_FAULTS; i++) {
        if (faultList[i].active) count++;
    }
    return count;
}