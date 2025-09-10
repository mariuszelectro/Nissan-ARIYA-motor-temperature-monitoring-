#include "light_sensor_control.h"
#include "Config.h"
#include "Arduino.h"

// Zmienne do kontroli odczytu jasności i jej adaptacji
static unsigned long lastLightReadTime = 0;
static unsigned long lastBrightnessChangeTime = 0;
static int highBrightnessCounter = 0;
static int lowBrightnessCounter = 0;

// Zmienne globalne zdefiniowane w innym miejscu, tutaj są tylko deklaracje
extern uint8_t currentBacklightPWM;

/**
 * @brief Odczytuje napięcie z fotorezystora.
 * @return Zmierzona wartość napięcia w woltach.
 */
static float readLightSensorVoltage() {
    int sensorValue = analogRead(LIGHT_SENSOR_PIN);
    // Używamy stałych z Config.h do przeliczenia na napięcie
    // Naprawiona formuła dla 12-bitowego ADC
    return ((float)sensorValue / MY_ADC_RESOLUTION_VALUE) * ADC_MAX_VOLTAGE;
}

/**
 * @brief Obsługuje odczyt z fotorezystora i dynamiczną regulację podświetlenia LCD.
 * @param displayData Wskaźnik do struktury danych wyświetlacza.
 * @param currentTime Aktualny czas w milisekundach.
 */
void handleBrightnessControl(DisplayData* displayData, unsigned long currentTime) {
    if (currentTime - lastLightReadTime >= LIGHT_READ_INTERVAL_MS) {
        int sensorValue = analogRead(LIGHT_SENSOR_PIN);
        // Wyświetl surowy odczyt ADC w celu diagnostyki
        Serial.printf("Surowy odczyt ADC: %d\n", sensorValue);
        
        float currentVoltage = ((float)sensorValue / MY_ADC_RESOLUTION_VALUE) * ADC_MAX_VOLTAGE;        
        lastLightReadTime = currentTime;

        // Aktualizuj pole napięcia w strukturze DisplayData
        displayData->MainVoltage.RealVoltage = currentVoltage;
        
        if (currentVoltage > 1.6) {
            highBrightnessCounter++;
            lowBrightnessCounter = 0;
        } else {
            lowBrightnessCounter++;
            highBrightnessCounter = 0;
        }

        if (highBrightnessCounter >= CONSECUTIVE_SAMPLES_THRESHOLD) {
            if (currentBacklightPWM != 255 && (currentTime - lastBrightnessChangeTime >= BRIGHTNESS_UPDATE_INTERVAL_MS)) {
                // Wywołanie globalnej funkcji zdefiniowanej w LCD_Display.cpp
                setBacklightPWM(255);
                lastBrightnessChangeTime = currentTime;
                highBrightnessCounter = 0;
            }
        } else if (lowBrightnessCounter >= CONSECUTIVE_SAMPLES_THRESHOLD) {
            if (currentBacklightPWM != (uint8_t)(255 * 0.30) && (currentTime - lastBrightnessChangeTime >= BRIGHTNESS_UPDATE_INTERVAL_MS)) {
                // Wywołanie globalnej funkcji zdefiniowanej w LCD_Display.cpp
                setBacklightPWM((uint8_t)(255 * 0.30));
                lastBrightnessChangeTime = currentTime;
                lowBrightnessCounter = 0;
            }
        }
        
        // Zaktualizuj procentową wartość jasności na podstawie aktualnej wartości PWM
        displayData->brightnessPercent = (int) (currentBacklightPWM / 255.0 * 100.0);
    }
}