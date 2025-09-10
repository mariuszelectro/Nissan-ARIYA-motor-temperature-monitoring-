#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <LSM6DS3.h>

/**
 * @brief Struktura do przechowywania odczytów z IMU.
 */
struct IMUData {
    float tilt;        // Nachylenie wzdłuż osi Y (do przodu/tyłu)
    float angle;       // Kąt wzdłuż osi X (na boki)
    float kalmanAngle; // Estymowany kąt z filtru Kalmana
    int samplesCollected; // Liczba zebranych próbek od ostatniego odczytu
};

/**
 * @brief Inicjalizuje czujnik IMU.
 * @return true jesli inicjalizacja powiodla sie, false w przeciwnym razie.
 */
bool imu_init();

/**
 * @brief Kalibruje czujnik IMU. Musi być wywoływana w pętli.
 * @return true jeśli kalibracja została zakończona, false w przeciwnym razie.
 */
bool Call_IMU_Calibration();

/**
 * @brief Odczytuje uśrednione dane z IMU i zwraca obliczone kąty.
 * @param data Wskaznik do struktury IMUData, ktora zostanie wypelniona danymi.
 * @return true jesli odczyt powiodl sie i dane sa poprawne, false w przeciwnym razie.
 */
bool readIMUData(IMUData* data);

/**
 * @brief Zbieranie danych i aktualizacja filtru Kalmana.
 * Nalezy wywolywac w petli glownej.
 */
void CollectImu(void);

#endif