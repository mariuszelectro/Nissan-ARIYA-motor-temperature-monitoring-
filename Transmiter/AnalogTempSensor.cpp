#include "AnalogTempSensor.h"
#include "Arduino.h"
#include <cmath>  // Wymagane do użycia NAN
#include <cstdio>
#include <algorithm>  // Wymagane do użycia std::sort

// Deklaracje globalnych zmiennych do zbierania próbek i do przechowywania ostatniej wartości
int rawAdcReadings[NUM_SAMPLES];
int ADCzailanie = 0;
float calibrateZasilenie = 0;
int probkiZasilnie = 0;
int readingIndex = 0;
float lastAnalogTemperature = NAN;  // Inicjalizacja wartością NAN

// Zmienne do przechowywania ostatnio przetworzonych danych
int lastFilteredAdcValue = 0;
float lastVoltage = NAN;

// Zmienne do pomiaru czasu przetwarzania i opóźnienia odczytu
unsigned long processingCompletionTime = 0;
unsigned long readLatencyTime = 0;

// Funkcja do inicjalizacji pinu ADC
void analogTempSensor_init() {
  pinMode(ANALOG_TEMP_SENSOR_PIN, INPUT);
  pinMode(POWER_SENSOR_PIN, INPUT);
  analogReadResolution(12);
}

// Funkcja do kalibracji
float getCalibratedTemperature_from_ADC(int adcValue) {
  float slope = (CAL_TEMP_2 - CAL_TEMP_1) / (CAL_ADC_2 - CAL_ADC_1);
  float offset = CAL_TEMP_1 - slope * CAL_ADC_1;
  return (slope * adcValue + offset);
}

// Funkcja do zbierania danych z czujnika
void collectTempProbe() {
  rawAdcReadings[readingIndex] = analogRead(ANALOG_TEMP_SENSOR_PIN);
  readingIndex++;

  if (readingIndex >= NUM_SAMPLES) {
    std::sort(rawAdcReadings, rawAdcReadings + NUM_SAMPLES);
    int middleIndex = NUM_SAMPLES / 2;
    int medianAdcValue = rawAdcReadings[middleIndex];
    lastFilteredAdcValue = medianAdcValue;

    lastVoltage = (float)medianAdcValue * ADC_VOLTAGE_MULTIPLIER;

    float finalTemperature = getCalibratedTemperature_from_ADC(medianAdcValue);

    if (finalTemperature < TEMP_LOWER_BOUND || finalTemperature > TEMP_UPPER_BOUND) {
      lastAnalogTemperature = NAN;
    } else {
      lastAnalogTemperature = finalTemperature;
    }

    processingCompletionTime = millis();
    readingIndex = 0;
  } 
  if(probkiZasilnie < 50){
    ADCzailanie += analogRead(POWER_SENSOR_PIN);
    probkiZasilnie++;
    if(probkiZasilnie==50)
    {
      ADCzailanie/=probkiZasilnie;
      float temp=(float) ADCzailanie/229.1;
      calibrateZasilenie=temp; 
      ADCzailanie=0;
      char tmep[32];
      sprintf(tmep,"zasilanie %d,%.3fV\n",probkiZasilnie,calibrateZasilenie);
      Serial.println(tmep);
    } 
  }
}
float PomiarZsailania() {
  probkiZasilnie = 0;
  return calibrateZasilenie;
}

// Funkcja do pobierania ostatniej przefiltrowanej wartosci temperatury
int getFilteredAnalogTemperature(float* inputVoltage) {
  readLatencyTime = millis() - processingCompletionTime;
  *inputVoltage = lastVoltage;

  // Wygodne wyświetlanie danych na Serial Monitorze
  char infoBuffer[100];
  snprintf(infoBuffer, sizeof(infoBuffer), "ADC info RAW%d, temp%d, samples%d, time%d",
           lastFilteredAdcValue, (int)(lastAnalogTemperature * 100), NUM_SAMPLES, (int)readLatencyTime);
  Serial.println(infoBuffer);

  // Zwraca ostatnią poprawnie obliczoną temperaturę pomnożoną przez 100
  // LUB -1, jeśli wartość jest niepoprawna (NAN), co umożliwi jej sprawdzenie.
  if (!isnan(lastAnalogTemperature)) {
    return (int)(lastAnalogTemperature * 100);
  } else {
    return -1;  // Wartość, która oznacza, że odczyt jest niepoprawny
  }
}