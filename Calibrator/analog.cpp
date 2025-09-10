#include "delay.h"
#include "analog.h"
#include <Arduino.h>

// Zmienna do przechowywania czasu życia w dniach
unsigned long bootTime = 0;

// Funkcja do inicjalizacji i ustawienia prądu ładowania
void setChargeCurrent_100mA() {
  pinMode(PIN_BAT_CHARGE_CURRENT, OUTPUT);
  digitalWrite(PIN_BAT_CHARGE_CURRENT, LOW);
}

// Funkcja do włączenia dzielnika napięcia
void enableVoltageDivider() {
   analogReadResolution(12);
  pinMode(PIN_VOLTAGE_DIVIDER_ENABLE, OUTPUT);
  digitalWrite(PIN_VOLTAGE_DIVIDER_ENABLE, LOW); // LOW aktywuje dzielnik
}
#define VBAT_PER_LBS (0.003515625F) // 3.6 reference and 10 bit resolution

// Funkcja do odczytu napięcia baterii

float getBatteryVoltage() {
  digitalWrite(VBAT_ENABLE, LOW);
  digitalWrite(PIN_VOLTAGE_DIVIDER_ENABLE, LOW);
  //char nameBuffer[32];
  int i;
  uint32_t rawADC=0;
  for(i=0;i<20;i++)
  {
    rawADC+= analogRead(PIN_VBAT);
    delayMicroseconds(100);
  }
  rawADC/=20;

  //sprintf(nameBuffer, "RAW ADC %d\n", rawADC);

  //Serial.print(nameBuffer);
  //float adcVoltage = adcCount * (VBAT_PER_LBS/2);
  //float vBat = adcVoltage * (1510.0 / 510.0);
  float voltage = (float)rawADC * (3.3 / 4096.0) * ( (float)1165 + (float)510 ) / (float)510;

  digitalWrite(VBAT_ENABLE, HIGH);
  return voltage;
}


// float getBatteryVoltage() {
//   // Włącz dzielnik napięcia na czas pomiaru
//   digitalWrite(PIN_VOLTAGE_DIVIDER_ENABLE, LOW);
//   delay(1); // Krótka przerwa na ustabilizowanie się napięcia
  
//   // Odczytaj surową wartość z ADC
//   int rawADC = analogRead(PIN_BAT_VOLTAGE);
  
//   // Wyłącz dzielnik, aby oszczędzać energię
//   digitalWrite(PIN_VOLTAGE_DIVIDER_ENABLE, HIGH);

//   // Konwersja na napięcie w V (używając stałej kalibracji 2.96 z Twojego oryginalnego kodu)
//   float voltage = (float)rawADC * (3.3 / 4095.0) * 2.96;

//   return voltage;
// }

// Funkcja do obliczania czasu życia w dniach
