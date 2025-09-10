#ifndef ANALOGTEMPSENSOR_H
#define ANALOGTEMPSENSOR_H

#include <Arduino.h>
#include <cmath> // Wymagane do uzycia NAN

// Definicje pinow
const int ANALOG_TEMP_SENSOR_PIN = A0;
const int POWER_SENSOR_PIN=A2;

// Stałe kalibracyjne dla funkcji liniowej, bazujace na surowych wartosciach ADC (0-4095)
const float CAL_ADC_1 = 68;   // Wartość ADC dla Punktu 1 (np. przy temperaturze 25.4°C)
const float CAL_TEMP_1 = 13.06;  // Temperatura dla Punktu 1
const float CAL_ADC_2 = 3134;   // Wartość ADC dla Punktu 2 (np. przy temperaturze 90.01°C)
const float CAL_TEMP_2 = 87.81;  // Temperatura dla Punktu 2

// Nowe stałe do weryfikacji zakresu temperatury
const float TEMP_LOWER_BOUND = 0.0;
const float TEMP_UPPER_BOUND = 120.0;

// Stałe do konwersji napięcia na temperaturę
const float ADC_VOLTAGE_MULTIPLIER = 3.3 / 4096.0;

// Nowe stałe do filtrowania
const int NUM_SAMPLES = 195; // Zmniejszono do 190, aby uniknąć desynchronizacji
const int SAMPLES_TO_DISCARD = 10; // Odrzucamy 10 z góry i 10 z dołu
#define TMR_READ_ANALOG_MS 5           //okres próbkowania ADC

// Deklaracje funkcji
void analogTempSensor_init();
void collectTempProbe(void);
int getFilteredAnalogTemperature(float* inputVoltage);
float PomiarZsailania();

#endif // ANALOGTEMPSENSOR_H