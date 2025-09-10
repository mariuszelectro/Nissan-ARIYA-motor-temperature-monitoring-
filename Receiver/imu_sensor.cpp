#include "imu_sensor.h"
#include <Wire.h>     // Niezbedne dla komunikacji I2C
#include <LSM6DS3.h>  // Biblioteka dla czujnika LSM6DS3
#include "Adafruit_Sensor_Calibration.h" // Poprawka: Dodano biblioteke do zapisu i odczytu kalibracji

// Utworzenie globalnej instancji klasy LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  // Adres I2C 0x6A dla LSM6DS3

// Instancja kalibratora
Adafruit_Sensor_Calibration_SDFat cal; // Poprawka: Dodano instancje do zapisu kalibracji

// Statyczne zmienne do przechowywania wartości kalibracji (offsetu)
static float roll_offset = 0.0;
static float pitch_offset = 0.0;

// --- Zmienne dla filtru Kalmana ---
static float kalman_angle = 0;   // Estymowany kat
static float kalman_bias = 0;    // Estymowany blad (bias) zyroskopu
static float kalman_P[2][2] = {{0, 0}, {0, 0}}; // Macierz kowariancji bledu

// Parametry filtru Kalmana (mozesz je dostroic)
static float Q_angle = 0.001; // Szum procesu (kata)
static float Q_bias = 0.003;  // Szum biasu
static float R_measure = 0.03; // Szum pomiaru (akcelerometru)

// --- Zmienne diagnostyczne ---
static int probe = 0; // Licznik wywolan CollectImu()

// --- Stan kalibracji (nieblokujacej) ---
static bool calibration_in_progress = false;
static int cal_sample_count = 0;
static float cal_accel_x_sum = 0.0;
static float cal_accel_y_sum = 0.0;
static float cal_accel_z_sum = 0.0;

// --- Implementacja filtru Kalmana ---

void Kalman_init(float initial_angle) {
    kalman_angle = initial_angle;
    kalman_bias = 0;
    kalman_P[0][0] = 1;
    kalman_P[0][1] = 0;
    kalman_P[1][0] = 0;
    kalman_P[1][1] = 1;
}

void Kalman_update(float newAngle, float newRate, float dt) {
    // 1. Faza predykcji
    kalman_angle += dt * (newRate - kalman_bias);
    kalman_P[0][0] += dt * (dt * kalman_P[1][1] - kalman_P[0][1] - kalman_P[1][0] + Q_angle);
    kalman_P[0][1] -= dt * kalman_P[1][1];
    kalman_P[1][0] -= dt * kalman_P[1][1];
    kalman_P[1][1] += dt * Q_bias;

    // 2. Faza korekcji
    float y = newAngle - kalman_angle; // Roznica miedzy pomiarem a estymacja
    float S = kalman_P[0][0] + R_measure;
    float K[2];
    K[0] = kalman_P[0][0] / S;
    K[1] = kalman_P[1][0] / S;

    kalman_angle += K[0] * y;
    kalman_bias += K[1] * y;

    float P00_temp = kalman_P[0][0];
    float P01_temp = kalman_P[0][1];
    kalman_P[0][0] -= K[0] * P00_temp;
    kalman_P[0][1] -= K[0] * P01_temp;
    kalman_P[1][0] -= K[1] * P00_temp;
    kalman_P[1][1] -= K[1] * P01_temp;
}

// --- Funkcje IMU ---

bool imu_init() {
  Wire.begin();
  if (myIMU.begin() != 0) {
    Serial.println("Device error - LSM6DS3TR-C not found during init!");
    return false;
  } else {
    Serial.println("LSM6DS3TR-C zainicjalizowany pomyslnie!");
    
    // Poprawka: Inicjalizacja biblioteki i proba odczytu danych
    cal.begin();
    if (cal.loadCalibration()) {
      Serial.println("Odczytano dane kalibracyjne z pamieci Flash.");
      // Bezposrednie odwolanie do zmiennych czlonkowskich
      roll_offset = atan2(cal.accel_zerog[1], cal.accel_zerog[2]) * 180.0 / PI;
      pitch_offset = atan2(-cal.accel_zerog[0], sqrt(cal.accel_zerog[1] * cal.accel_zerog[1] + cal.accel_zerog[2] * cal.accel_zerog[2])) * 180.0 / PI;
      Serial.print("Offset Roll: "); Serial.println(roll_offset);
      Serial.print("Offset Pitch: "); Serial.println(pitch_offset);
    } else {
      Serial.println("Brak danych kalibracyjnych w pamieci. Wymagana kalibracja!");
    }
    
    // Inicjalizacja filtru Kalmana poczatkowym odczytem
    float initial_accel_y = myIMU.readFloatAccelY();
    float initial_accel_z = myIMU.readFloatAccelZ();
    float initial_angle = atan2(initial_accel_y, initial_accel_z) * 180.0 / PI;
    Kalman_init(initial_angle);
    
    return true;
  }
}

bool Call_IMU_Calibration() {
    if (!calibration_in_progress) {
        Serial.println(F("Rozpoczeto kalibracje. Trzymaj uklad nieruchomo."));
        calibration_in_progress = true;
        cal_sample_count = 0;
        cal_accel_x_sum = 0.0;
        cal_accel_y_sum = 0.0;
        cal_accel_z_sum = 0.0;
    }

    float current_accel_x = myIMU.readFloatAccelX();
    float current_accel_y = myIMU.readFloatAccelY();
    float current_accel_z = myIMU.readFloatAccelZ();

    if (!isnan(current_accel_x) && !isnan(current_accel_y) && !isnan(current_accel_z)) {
        cal_accel_x_sum += current_accel_x;
        cal_accel_y_sum += current_accel_y;
        cal_accel_z_sum += current_accel_z;
        cal_sample_count++;
    }

    if (cal_sample_count >= 200) { // Zbieramy 200 probek do kalibracji
        if (cal_sample_count > 0) {
            float avg_accel_y = cal_accel_y_sum / cal_sample_count;
            float avg_accel_z = cal_accel_z_sum / cal_sample_count;
            float avg_accel_x = cal_accel_x_sum / cal_sample_count;
            
            roll_offset = atan2(avg_accel_y, avg_accel_z) * 180.0 / PI;
            pitch_offset = atan2(-avg_accel_x, sqrt(avg_accel_y * avg_accel_y + avg_accel_z * avg_accel_z)) * 180.0 / PI;

            // Poprawka: Zapisanie danych kalibracji do pamieci
            cal.accel_zerog[0] = avg_accel_x;
            cal.accel_zerog[1] = avg_accel_y;
            cal.accel_zerog[2] = avg_accel_z;
            cal.saveCalibration();

            Serial.println("Kalibracja IMU zakonczona. Biezaca pozycja ustawiona jako zero.");
            Serial.println("Zapisano dane kalibracyjne do pamieci Flash.");
            Serial.print("Offset Roll: "); Serial.println(roll_offset);
            Serial.print("Offset Pitch: "); Serial.println(pitch_offset);
        } else {
            Serial.println("Kalibracja IMU nie powiodla sie.");
            roll_offset = 0.0;
            pitch_offset = 0.0;
        }
        calibration_in_progress = false;
        return true;
    }
    return false;
}

bool readIMUData(IMUData* data) {
    if (!data) {
        return false;
    }
    
    // Zabezpieczenie przed jednoczesnym odczytem i zapisem
    noInterrupts();
    int temp_probe = probe;
    probe = 0; // Zresetuj licznik
    interrupts();

    // Zwraca przefiltrowany kat
    data->kalmanAngle = kalman_angle;
    data->samplesCollected = temp_probe;
    
    // Zastosowanie kalibracji
    data->angle = data->kalmanAngle - roll_offset;
    
    // Odczyt surowych danych do obliczenia tilt
    float accel_x = myIMU.readFloatAccelX();
    float accel_y = myIMU.readFloatAccelY();
    float accel_z = myIMU.readFloatAccelZ();
    if (isnan(accel_x) || isnan(accel_y) || isnan(accel_z)) {
        return false;
    }
    data->tilt = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI - pitch_offset;
    char temp[30];
    sprintf(temp,"Gyro probe %d\n",temp_probe);
    Serial.println(temp);
    
    return true;
}

void CollectImu(void) {
    static unsigned long last_read_time = 0;
    const unsigned long read_interval = 20; // 20 ms
    unsigned long current_time = millis();

    if (current_time - last_read_time >= read_interval) {
        last_read_time = current_time;

        // Czas, ktory uplynal od ostatniej aktualizacji (w sekundach)
        float dt = (float)read_interval / 1000.0;

        // Odczyt surowych danych
        float accel_x = myIMU.readFloatAccelX();
        float accel_y = myIMU.readFloatAccelY();
        float accel_z = myIMU.readFloatAccelZ();
        float gyro_x = myIMU.readFloatGyroX();
        
        if (isnan(accel_x) || isnan(accel_y) || isnan(accel_z) || isnan(gyro_x)) {
            return;
        }
        
        // Obliczenie kata z akcelerometru
        float accel_roll = atan2(accel_y, accel_z) * 180.0 / PI;

        // Przekazanie danych do filtru Kalmana
        Kalman_update(accel_roll, gyro_x, dt);
        
        // Inkrementuj licznik diagnostyczny
        probe++;
    }
}