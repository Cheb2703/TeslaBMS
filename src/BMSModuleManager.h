#pragma once
#include "config.h"
#include "BMSModule.h"
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include "Displaymanager.h"
#include <ArduinoJson.h>
//#include <esp32_can.h>

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
};

class BMSModuleManager
{
public:
    BMSModuleManager(AsyncWebServer* webServer);
    //BMSModuleManager();
    void balanceCells();
    void balanceCell(int cellNumber);
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void readSetpoints();
    void setBatteryID();
    float getPackVoltage();
    float getAvgTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighestModuleVolt();
    void printPackSummary();
    void printPackDetails();
    void printJsonData();
    String buildJsonData();
    void publishIndividualData(PubSubClient& client, const char* baseTopic, const String systemName);
    void handleBatteryStats(AsyncWebServerRequest* request, const String& bmsJson);
    void sendBatteryStats(String systemName, String ftpServer, String ftpUser, String ftpPassword, const String& bmsJson);
    void broadcastBatteryStats(AsyncWebSocket* ws, const String& bmsJson);

    void buildDisplayData(DisplayData& out);
    void setDisplay(DisplayManager* d) { _display = d; }

    // Fault registry -- reportFault()/clearFaultById() are also called from
    // main.cpp for fault sources that live outside this class (the hardware
    // FAULT pin, and the serial console's manual test-fault injection), so
    // every fault source ends up in one unified list regardless of origin.
    void reportFault(const char* id, const char* reason);
    void clearFaultById(const char* id);
    int  getActiveFaultCount();

private:
    float packVolt;                         // All modules added together
    float lowestPackVolt;
    float highestPackVolt;
    void publishSensorData(PubSubClient& client, const char* baseTopic, const String systemName, const String& cellID, const String& devClass, const String& unit, const int precision, const String& value);
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.

    FaultRecord faultList[MAX_ACTIVE_FAULTS];
    uint8_t commFails[MAX_MODULE_ADDR + 1]; // consecutive failed read cycles per module (see BMB_COMM_FAIL_LIMIT)
    int CellsBalancing;
    AsyncWebServer* server;
    
    //void sendBatterySummary(String systemName, String ftpServer, String ftpUser, String ftpPassword);
    //void sendModuleSummary(int module);
    void sendCellDetails(int module, int cell);

    DisplayManager* _display;   // pointer, set via setDisplay()
    
};