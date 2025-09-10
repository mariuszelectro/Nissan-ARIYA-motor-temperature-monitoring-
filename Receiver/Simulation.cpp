#include "Simulation.h"
#include <Arduino.h>
#include "LCD_Display.h" 
#include "Config.h"

// Zmienne statyczne do przechowywania stanu symulacji
static float sim_testTemp = 25.0; 
static unsigned long temp_start_time = 0;
static int sim_testHeight = SIM_HEIGHT_MIN;
static bool isHeightIncreasing = true;
static unsigned long lastHeightChangeTime = 0;
static int sim_testAngle = SIM_ANGLE_MIN;
static bool isAngleIncreasing = true;
static unsigned long lastAngleChangeTime = 0;
static unsigned long last_calibration_toggle_time = 0;
static bool is_calibration_active = false;
static unsigned long calibration_start_time = 0;
static unsigned long last_data_missing_toggle_time = 0;
static bool is_data_missing = false;
static unsigned long lastVoltageChangeTime = 0;
static bool isHighVoltage = false;
static unsigned long lastRssiChangeTime = 0;
static int sim_testRssi = -70;
static unsigned long lastSourceChangeTime = 0;
static bool isAnalogSource = true;

// ZMIENNE DO OBSŁUGI AUTOMATÓW STANÓW ALARMU I WENTYLATORA
static unsigned int alarm_state_counter = 0;
static unsigned int wentylator_state_counter = 0;
static float last_valid_temp = 25.0; // Przechowuje ostatnią poprawną temperaturę
static char alarm_state_internal = ' '; // Wewnętrzny stan automatu alarmu
static char wentylator_state_internal = ' '; // Wewnętrzny stan automatu wentylatora


#define STATE_CHANGE_COUNT 7

// === MODULARNE FUNKCJE SYMULACJI ===

static void simulateTemperature(DisplayData* displayData, unsigned long currentTime) {
    if (temp_start_time == 0) {
        temp_start_time = currentTime;
    }
    
    unsigned long time_elapsed = currentTime - temp_start_time;
    sim_testTemp = 25.0 + ((SIM_TEMP_MAX_TEMP - 25.0) / SIM_TEMP_INCREASE_DURATION_MS) * time_elapsed;
    
    if (sim_testTemp > SIM_TEMP_MAX_TEMP) {
      sim_testTemp = 25.0;
      temp_start_time = currentTime;
    }
    
    displayData->MainTemperature.temperature = sim_testTemp;
}

static void simulateAlarmStates(DisplayData* displayData) {
    if (sim_testTemp >= ALARM_TEMP_THRESHOLD) {
        alarm_state_internal = 'A';
        alarm_state_counter = 0;
    } else if (is_data_missing) {
        alarm_state_internal = '!';
        alarm_state_counter = 0;
    } else {
        if (alarm_state_internal == 'A') {
            alarm_state_internal = 'N';
            alarm_state_counter = 1;
        } else if (alarm_state_internal == 'N') {
            if (alarm_state_counter >= STATE_CHANGE_COUNT) {
                alarm_state_internal = '!';
                alarm_state_counter = 1;
            } else {
                alarm_state_counter++;
            }
        } else if (alarm_state_internal == '!') {
            if (alarm_state_counter >= STATE_CHANGE_COUNT) {
                alarm_state_internal = ' ';
                alarm_state_counter = 0;
            } else {
                alarm_state_counter++;
            }
        }
    }
    displayData->Alarm.state = alarm_state_internal;
}

static void simulateWentylator(DisplayData* displayData) {
    if (sim_testTemp >= WENTYLATOR_TEMP_THRESHOLD) {
        wentylator_state_internal = 'W';
        wentylator_state_counter = 0;
    } else if (is_data_missing) {
        wentylator_state_internal = '!';
        wentylator_state_counter = 0;
    } else {
        if (wentylator_state_internal == 'W') {
            wentylator_state_internal = 'n';
            wentylator_state_counter = 1;
        } else if (wentylator_state_internal == 'n') {
            if (wentylator_state_counter >= STATE_CHANGE_COUNT) {
                wentylator_state_internal = '!';
                wentylator_state_counter = 1;
            } else {
                wentylator_state_counter++;
            }
        } else if (wentylator_state_internal == '!') {
            if (wentylator_state_counter >= STATE_CHANGE_COUNT) {
                wentylator_state_internal = ' ';
                wentylator_state_counter = 0;
            } else {
                wentylator_state_counter++;
            }
        }
    }
    displayData->Wentylator.state = wentylator_state_internal;
}

static void simulateHeight(DisplayData* displayData, unsigned long currentTime) {
    if (currentTime - lastHeightChangeTime >= HEIGHT_CHANGE_INTERVAL_MS) {
        lastHeightChangeTime = currentTime;
        if (isHeightIncreasing) {
            sim_testHeight += 100;
            if (sim_testHeight >= SIM_HEIGHT_MAX) {
                isHeightIncreasing = false;
            }
        } else {
            sim_testHeight -= 100;
            if (sim_testHeight <= SIM_HEIGHT_MIN) {
                isHeightIncreasing = true;
            }
        }
    }
    displayData->MainHeight.RealHeght = is_data_missing ? -1 : sim_testHeight;
}

static void simulateAngle(DisplayData* displayData, unsigned long currentTime) {
    if (currentTime - lastAngleChangeTime >= ANGLE_CHANGE_INTERVAL_MS) {
        lastAngleChangeTime = currentTime;
        if (isAngleIncreasing) {
            sim_testAngle += 5;
            if (sim_testAngle >= SIM_ANGLE_MAX) {
                isAngleIncreasing = false;
            }
        } else {
            sim_testAngle -= 5;
            if (sim_testAngle <= SIM_ANGLE_MIN) {
                isAngleIncreasing = true;
            }
        }
    }
    displayData->MainAngle.RealAngle = is_data_missing ? -1 : sim_testAngle;
}

static float simulateVoltage() {
    if (millis() - lastVoltageChangeTime >= VOLTAGE_CHANGE_INTERVAL_MS) {
        lastVoltageChangeTime = millis();
        isHighVoltage = !isHighVoltage;
    }
    return isHighVoltage ? 2.7 : 1.1;
}

static void simulateRssi(DisplayData* displayData, unsigned long currentTime) {
    if (!is_data_missing) {
        if (currentTime - lastRssiChangeTime >= RSSI_CHANGE_INTERVAL_MS) {
            lastRssiChangeTime = currentTime;
            sim_testRssi += random(-5, 6);
            sim_testRssi = constrain(sim_testRssi, -90, -30);
        }
        displayData->MainRssi.RealRssi = sim_testRssi;
    } else {
        displayData->MainRssi.RealRssi = -1;
    }
}

static void handleCalibrationSimulation(DisplayData* displayData, unsigned long currentTime) {
    if (!is_calibration_active && (currentTime - last_calibration_toggle_time >= CALIBRATION_INTERVAL_MS)) {
        is_calibration_active = true;
        calibration_start_time = currentTime;
        last_calibration_toggle_time = currentTime;
    }

    if (is_calibration_active && (currentTime - calibration_start_time >= CALIBRATION_DURATION_MS)) {
        is_calibration_active = false;
    }
    
    displayData->MainCalibrate.RealCalibrate = is_calibration_active;
}

static void simulateTemperatureSource(DisplayData* displayData, unsigned long currentTime) {
  if (currentTime - lastSourceChangeTime >= SOURCE_CHANGE_INTERVAL_MS) {
    isAnalogSource = !isAnalogSource;
    lastSourceChangeTime = currentTime;
  }
  displayData->MainTempeSource.AnalogDigital = isAnalogSource ? 'A' : 'C';
}

// === GŁÓWNA FUNKCJA SYMULACJI ===
void simulateSensorData(DisplayData* displayData, unsigned long currentTime) {
    if (currentTime - last_data_missing_toggle_time >= (is_data_missing ? DATA_MISSING_ON_INTERVAL_MS : DATA_MISSING_OFF_INTERVAL_MS)) {
        is_data_missing = !is_data_missing;
        last_data_missing_toggle_time = currentTime;
    }
    
    // if (!is_data_missing) {
    //     simulateTemperature(displayData, currentTime);
    //     last_valid_temp = displayData->MainTemperature.temperature;
    // } else {
    //     displayData->MainTemperature.temperature = last_valid_temp;
    // }

    if (displayData->Alarm.source == Simul) {
        simulateAlarmStates(displayData);
    }
    if (displayData->Wentylator.source == Simul) {
        simulateWentylator(displayData);
    }
    
    if (displayData->MainHeight.source == Simul) {
      simulateHeight(displayData, currentTime);
    }
    if (displayData->MainAngle.source == Simul) {
      simulateAngle(displayData, currentTime);
    }
    if (displayData->MainRssi.source == Simul) {
      simulateRssi(displayData, currentTime);
    }
    if (displayData->MainVoltage.source == Simul) {
      displayData->MainVoltage.RealVoltage = simulateVoltage();
    }
    if (displayData->MainCalibrate.Source == Simul) {
      handleCalibrationSimulation(displayData, currentTime);
    }
    if (displayData->MainTempeSource.source == Simul) {
      simulateTemperatureSource(displayData, currentTime); 
    }
}

void debugPrintSimulatedData(DisplayData* displayData) {
    char debug_buffer[300];
    char* ptr = debug_buffer;
    ptr += sprintf(ptr, "Symulacja:");

    bool first_entry = true;

    if (displayData->MainTemperature.source == Simul) {
        ptr += sprintf(ptr, " temp: %.2fC", displayData->MainTemperature.temperature);
        first_entry = false;
    }
    
    if (displayData->MainHeight.source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " height: %d", displayData->MainHeight.RealHeght);
        first_entry = false;
    }

    if (displayData->MainAngle.source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " angle: %d", displayData->MainAngle.RealAngle);
        first_entry = false;
    }

    if (displayData->MainVoltage.source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " voltage: %.2fV", displayData->MainVoltage.RealVoltage);
        first_entry = false;
    }

    if (displayData->MainRssi.source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " rssi: %d", displayData->MainRssi.RealRssi);
        first_entry = false;
    }

    if (displayData->MainCalibrate.Source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " cal: %d", displayData->MainCalibrate.RealCalibrate);
        first_entry = false;
    }
    
    if (displayData->Alarm.source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " alarm: %c", displayData->Alarm.state);
        first_entry = false;
    }

    if (displayData->Wentylator.source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " wentylator: %c", displayData->Wentylator.state);
        first_entry = false;
    }

    if (displayData->MainTempeSource.source == Simul) {
        if (!first_entry) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, " src_type: %c", displayData->MainTempeSource.AnalogDigital);
        first_entry = false;
    }

    Serial.println(debug_buffer);
}