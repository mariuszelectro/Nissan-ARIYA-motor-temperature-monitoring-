#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <LSM6DS3.h>


//* Flters Params*//
#define Q_angle_set 0.0005          //lower value = more stable bult higer latency 0.001
#define Q_bias_set 0.003            //gyro dift
#define R_measure_set 0.3           //How trust accel higer= trust more gyro 0.03

#define R_measure_high_set 0.8      //0,5 more = moer trust gyro 0,03-1,2
#define R_measure_low_set 0.09      //0,03 sesivity when no acceelration to  higer= more stable
#define GYRO_THRESHOLD_set  0.8     //1.0 accereartion limit to usec for calculation  less= trust gyro 
#define ACCEL_TOLERANCE_set 0.3      //0.2  huher = more smouth
#define ACCEL_LPF_ALPHA_set 0.05      //lower value = more filtererd 0,2



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
bool Call_IMU_Calibration(int write);

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

bool LoadImuCalibration(void);
void setCallIMUasZero(void);

#endif