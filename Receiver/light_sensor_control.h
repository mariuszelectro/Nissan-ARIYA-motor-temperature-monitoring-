#ifndef LIGHT_SENSOR_CONTROL_H
#define LIGHT_SENSOR_CONTROL_H

#include "LCD_Display.h" // Konieczne do uzycia struktury DisplayData


// Konfiguracja regulacji podswietlenia
//#define LIGHT_READ_INTERVAL_MS 1000            // Odczyt co 1 sekunde
//#define CONSECUTIVE_SAMPLES_THRESHOLD 5       // Zmiana po 5 kolejnych odczytach
//#define BRIGHTNESS_UPDATE_INTERVAL_MS 5000    // Opoznienie miedzy zmianami jasnosci

// Deklaracja funkcji do obslugi kontroli jasnosci
void handleBrightnessControl(DisplayData* displayData, unsigned long currentTime);

#endif // LIGHT_SENSOR_CONTROL_H