#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Definicje Pinow dla XIAO nRF52840 ---
#define TFT_CS                  3   
#define TFT_DC                  6   
#define TFT_RST                 7   
#define TFT_BL                  D0  
#define LIGHT_SENSOR_PIN        A1  // Prawidlowe uzycie pinu A1/D1 dla fotorezystora
#define COOLING_PIN             D5  
#define LED_RED_PIN             LED_BUILTIN 
#define LED_BLUE_PIN            LEDB        
#define BUZZER_PIN              D9

// --- Konfiguracja ADC ---
const float ADC_MAX_VOLTAGE = 3.3; 
const int MY_ADC_RESOLUTION_VALUE = 4096; 

// --- Konfiguracja trybow pracy i kalibracji ---
const unsigned long CALIBRATION_INTERVAL_MS = 40000;
const unsigned long CALIBRATION_DURATION_MS = 1000; 
const unsigned long NORMAL_DURATION_MS = 9000;      

// --- Konfiguracja odczytow jasnosci i jej adaptacji ---
const unsigned long LIGHT_READ_INTERVAL_MS = 1000; 
const unsigned long BRIGHTNESS_UPDATE_INTERVAL_MS = 5000;
const int CONSECUTIVE_SAMPLES_THRESHOLD = 5; 

// --- Stałe progowe dla wentylatora i alarmu ---
#define WENTYLATOR_TEMP_THRESHOLD 66.0 // Stała temperatura włączenia wentylatora
#define ALARM_TEMP_THRESHOLD      70.0 // Stała temperatura włączenia alarmu

#endif