// bmp280_sensor.h
#ifndef BMP280_SENSOR_H
#define BMP280_SENSOR_H

#include <Arduino.h>           // Standardowe definicje Arduino
#include <Adafruit_BMP280.h>   // Biblioteka dla czujnika BMP280

// Struktura do przechowywania danych z czujnika BMP280
struct Bmp280Data {
    float temperature; // Temperatura w stopniach Celsjusza
    float pressure;    // Cisnienie w Pascalach (Pa)
    float altitude;
};

/**
 * @brief Inicjalizuje czujnik BMP280.
 * @return true jesli inicjalizacja powiodla sie, false w przeciwnym wypadku.
 */
bool initBmp280();

/**
 * @brief Odczytuje dane z czujnika BMP280 i zapisuje je do struktury.
 * @param data Wskaznik do struktury Bmp280Data, ktora zostanie wypelniona odczytanymi danymi.
 * @return true jesli odczyt powiodl sie, false w przeciwnym wypadku.
 */
bool readBmp280Data(Bmp280Data* data);
void setAbsoluteHighest(int meters);

#endif // BMP280_SENSOR_H