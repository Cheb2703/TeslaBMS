#pragma once
#include "config.h"
#include "BMSModule.h"
#include "Displaymanager.h"

extern int numFoundModules;                    // The number of modules that seem to exist
extern float lowestCellVolt;
extern float highestCellVolt;
extern float lowestPackTemp;
extern float highestPackTemp;

// A single tracked fault -- identified by a stable short id so the same
// underlying condition (e.g. "module 1 cell 4 over-voltage") is recognized
// as ONE ongoing fault across repeated polls, rather than a new fault each
// time. This is what lets the display show "how long has this been going on"
// per fault, and show multiple simultaneous faults instead of only the most
// recent one.
#define MAX_ACTIVE_FAULTS 16
struct FaultRecord {
    bool     active;
    char     id[12];       // stable key, e.g. "M1C4OV", "M2UT", "M1REG", "HWPIN", "TESTFLT"
    char     reason[40];   // human-readable description shown on the LCD/serial
    uint32_t startMillis;  // when this specific fault first became active
    bool     blocksCharger; // true = charger is held off while this is active; false = alarm only (e.g. low cell voltage)
};

class BMSModuleManager
{
public:
    BMSModuleManager();
    void balanceCells(bool refreshReadings = true);   // false = use the values getAllVoltTemp() just read
    void balanceCell(int cellNumber);
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    void clearFaults();
    void getAllVoltTemp();
    float getPackVoltage();
    float getAvgTemperature();
    float getAvgCellVolt();
    void printProtectionSettings();   // console 'k': each module's own OV/UV/OT trip points (read-only)
    void printPackSummary();
    void printPackDetails();
    void printJsonData();

    void buildDisplayData(DisplayData& out);
    void setDisplay(DisplayManager* d) { _display = d; }

    // Fault registry -- reportFault()/clearFaultById() are also called from
    // main.cpp for fault sources that live outside this class (the hardware
    // FAULT pin, and the serial console's manual test-fault injection), so
    // every fault source ends up in one unified list regardless of origin.
    void reportFault(const char* id, const char* reason, bool blocksCharger = true);
    void clearFaultById(const char* id);
    int  getActiveFaultCount();
    int  getBlockingFaultCount();   // active faults that hold the charger off (see FaultRecord::blocksCharger)

private:
    float packVolt;                         // All modules added together
    float lowestPackVolt;
    float highestPackVolt;
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.

    FaultRecord faultList[MAX_ACTIVE_FAULTS];
    uint8_t commFails[MAX_MODULE_ADDR + 1]; // consecutive failed read cycles per module (see BMB_COMM_FAIL_LIMIT)
    int CellsBalancing;

    DisplayManager* _display;   // pointer, set via setDisplay()
    
};