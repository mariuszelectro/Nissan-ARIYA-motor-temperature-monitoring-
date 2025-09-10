#ifndef BLE_CALIBRATION_H
#define BLE_CALIBRATION_H

#include "LCD_Display.h" 
#include <bluefruit.h>

/**
 * @brief Funkcja do sprawdzania i wyzwalania kalibracji za pomocą beacona.
 * * Ta funkcja zarządza logiką jednorazowego wykrycia
 * beacona kalibracyjnego, tak aby nie ingerowac w glowny ble_control.cpp.
 *
 * @param adv Wskaźnik do obiektu reklamy BLE.
 * @param data Wskaźnik do globalnej struktury DisplayData.
 */
void checkAndCalibrateWithBeacon(const uint8_t* data, uint8_t len, int8_t rssi);

#endif