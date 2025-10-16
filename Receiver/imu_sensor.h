#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <LSM6DS3.h>

// Nowa struktura do zapisu kalibracji, uwzględniająca offsety Grawitacji dla osi X i Z
typedef struct {
    float roll;
    float pitch;
    float g_x_offset; // Statyczna składowa Grawitacji na osi X (powinna być bliska 1G)
    float g_z_offset; // Statyczna składowa Grawitacji na osi Z (np. 0.3G)
} calibration_data_t;


// * Filters Params - Ustawienia dla SZYBKIEJ KOREKCJI DRYFU i ODRZUCENIA ACCEL (Preset 8) *//

#define KalmmaPresets 8

#if (KalmmaPresets==8)
// -----------------------------------------------------------------
// PARAMETRY OGÓLNE I LPF (ADAPTOWANE DO SAMOCHODU)
// -----------------------------------------------------------------
#define ACCEL_LPF_ALPHA_set     0.1f    // Umiarkowana filtracja
#define ACCEL_TOLERANCE_set     0.35f   // Próg Normy: Podniesiony, aby ignorować wibracje samochodu (do 0.35g)
#define GYRO_THRESHOLD_set      5.0f    // Próg: Wyższy niż stały dryf (5.0 deg/s) na stole zmierzono ok 4 deg/s
// -----------------------------------------------------------------
// TRYB KOREKCJA (Postój) - Duże zaufanie do Accel
// -----------------------------------------------------------------
#define R_measure_low_set       0.05f   // Mocne zaufanie do akcelerometru (niski R)
#define Q_bias_low_set          0.003f  // Umiarkowana estymacja dryfu
#define Q_angle_set             0.005f  // Umiarkowana ufność do modelu kąta
// -----------------------------------------------------------------
// TRYB DYNAMIKA (Ruch) - KLUCZOWE ZMIANY dla WIBRACJI SAMOCHODU
// -----------------------------------------------------------------
#define R_measure_high_set      50.0f  // KLUCZOWY: CAŁKOWITE IGNOROWANIE Accel (bardzo wysoki R, ignoruje wibracje)
#define Q_bias_high_set         0.07f   // Agresywna estymacja dryfu
// -----------------------------------------------------------------
// DEFAULTY
// -----------------------------------------------------------------
#define R_measure_set           R_measure_low_set
#endif

/**
 * @brief Struktura do przechowywania odczytów z IMU.
 */
struct IMUData {
    float tilt;         // Nachylenie wzdłuż osi Z (do przodu/tyłu) - PITCH
    float angle;        // Kąt wzdłuż osi Y (na boki) - ROLL
    float kalmanAngle;  // Estymowany kąt z filtru Kalmana
    int samplesCollected; // Liczba zebranych próbek od ostatniego odczytu
    float X; // Średnie przyspieszenie X
    float Y; // Średnie przyspieszenie Y
    float Z; // Średnie przyspieszenie Z
};

/**
 * @brief Inicjalizuje czujnik IMU.
 */
bool imu_init();

/**
 * @brief Kalibruje czujnik IMU. Musi być wywoływana w pętli.
 * @param write Czy zapisac dane kalibracji do pamieci flash.
 * @return true jesli kalibracja została zakończona, false w przeciwnym razie.
 */
bool Call_IMU_Calibration(int write);

/**
 * @brief Odczytuje uśrednione dane z IMU i zwraca obliczone kąty.
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