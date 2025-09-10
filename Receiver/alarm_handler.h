#ifndef ALARM_HANDLER_H
#define ALARM_HANDLER_H

#include <Arduino.h>
#include "LCD_Display.h"

// ### KONFIGURACJA BUZERA I ALARMU ###
// Włącz lub wyłącz dźwięk alarmu poprzez odkomentowanie/zakomentowanie poniższej linii.
//#define NoSound // Głos alarmu WYŁĄCZONY.

// Definicje stałych specyficznych dla logiki alarmu
const int BUZZER_FREQ_HZ = 1000;
const float ALARM_MAX_TEMP_FOR_BEEP = 100.0;

void handleAlarm(DisplayData* displayData, unsigned long currentTime, bool isButtonPushed);
#endif // ALARM_HANDLER_H