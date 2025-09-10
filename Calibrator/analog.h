#ifndef ANALOG_H
#define ANALOG_H

#include <Arduino.h>
#include <cmath>

// Definicje pinow
const int PIN_BAT_CHARGE_CURRENT = PIN_CHARGING_CURRENT;
const int PIN_BAT_VOLTAGE = PIN_VBAT;
const int PIN_VOLTAGE_DIVIDER_ENABLE = VBAT_ENABLE;


// Definicje pinow dla Seeed Studio XIAO nRF52840 Sense (mbed core)
/*const int PIN_BAT_CHARGE_CURRENT = 15;
const int PIN_BAT_VOLTAGE = 17;
const int PIN_VOLTAGE_DIVIDER_ENABLE = 16;
*/
// Stałe do konwersji napięcia na temperaturę
const float ADC_VOLTAGE_MULTIPLIER = 3.3 / 4096.0;

// Deklaracje funkcji
void setChargeCurrent_100mA();
void enableVoltageDivider();
float getBatteryVoltage();


#endif // ANALOG_H