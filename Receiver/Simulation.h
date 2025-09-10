#ifndef SIMULATION_H
#define SIMULATION_H

#include "LCD_Display.h"
#include "Config.h" 

// === Definicje zachowania symulacji ===
#define SIM_TEMP_INCREASE_DURATION_MS 120000 
#define SIM_TEMP_MAX_TEMP 120.0 

// === DEFINICJE DLA WYSOKOSCI I KATA ===
#define SIM_HEIGHT_MIN 0
#define SIM_HEIGHT_MAX 2500
#define SIM_ANGLE_MIN 0
#define SIM_ANGLE_MAX 99

// === Interwały czasowe dla symulacji ===
#define DATA_MISSING_ON_INTERVAL_MS   1000 
#define DATA_MISSING_OFF_INTERVAL_MS  30000

// === Stałe związane ze zmianami danych ===
#define VOLTAGE_CHANGE_INTERVAL_MS 5000 
#define RSSI_CHANGE_INTERVAL_MS 1000 

// === WARTOSCI SKOKOW DANYCH DLA SYMULACJI ===
#define HEIGHT_CHANGE_INTERVAL_MS 500
#define ANGLE_CHANGE_INTERVAL_MS 2000
#define SIM_RSSI_INTERVAL_MS 1000

// *** Progi specyficzne dla symulacji ***
#define WENTYLATOR_TEMP_THRESHOLD 66.0 
#define ALARM_TEMP_THRESHOLD 70.0      

// === WARTOSCI SKOKOW DANYCH DLA SYMULACJI ===
const float TEMP_JUMP = 0.9;
const int HEIGHT_JUMP = 33;
const int ANGLE_JUMP = 1;
const int RSSI_JUMP = 3;
const float VOLTAGE_JUMP = 0.7;
const unsigned long SOURCE_CHANGE_INTERVAL_MS = 5000; 
const int BUZZER_FREQUENCY = 1000; 

// NOWA WERSJA 
void simulateSensorData(DisplayData* displayData, unsigned long currentTime);
void debugPrintSimulatedData(DisplayData* displayData);

#endif