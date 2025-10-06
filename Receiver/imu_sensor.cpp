#include "imu_sensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "ConfigurationStorage.h"

#define CTRL8_XL 0x17
#define CTRL6_C 0x15
#define CTRL7_G 0x16
#define LSM6DS3_ADDR 0x6A
#define CALIBRATION_BLOCK 0

LSM6DS3 myIMU(I2C_MODE, LSM6DS3_ADDR);

static float roll_offset = 0.0;
static float pitch_offset = 0.0;

static const float default_roll_offset = 0.0;
static const float default_pitch_offset = 0.0;

typedef struct {
    float roll;
    float pitch;
} calibration_data_t;

static float kalman_angle = 0;
static float kalman_bias = 0;
static float kalman_P[2][2] = { { 0, 0 }, { 0, 0 } };

// Zmienna R_measure jest teraz zmieniana dynamicznie
static float R_measure = R_measure_set;

// Stale Q i R zdefiniowane w naglowku dla aktualnego Presetu
#if (KalmmaPresets>=7)
static const float Q_angle_base = Q_angle_set;
static const float Q_bias_low = Q_bias_low_set;
static const float Q_bias_high = Q_bias_high_set;
static const float ACCEL_Z_THRESHOLD = ACCEL_Z_THRESHOLD_set;
#else
static const float Q_angle_base = Q_angle_set;
static const float Q_bias_low = Q_bias_set; 
static const float Q_bias_high = Q_bias_set; 
#endif

static const float R_measure_high = R_measure_high_set;
static const float R_measure_low = R_measure_low_set;
static const float GYRO_THRESHOLD = GYRO_THRESHOLD_set;
static const float ACCEL_TOLERANCE = ACCEL_TOLERANCE_set; 
static const float ACCEL_LPF_ALPHA = ACCEL_LPF_ALPHA_set; 

static int probe = 0;
static float X=0;
static float Y=0;
static float Z=0;

static float filtered_accel_x = 0;
static float filtered_accel_z = 0;

static void initHWFilter() {
    Serial.println("Konfiguracja filtrow sprzetowych IMU...");
    uint8_t ctrl8_xl_val = 0;
    ctrl8_xl_val |= (0b01 << 6); 
    ctrl8_xl_val |= (1 << 0);
    
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(CTRL8_XL);
    Wire.write(ctrl8_xl_val); 
    Wire.endTransmission();
    
    uint8_t ctrl7_g_val = 0;
    ctrl7_g_val |= (1 << 4);
    ctrl7_g_val |= (0b000 << 0); 
    
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(CTRL7_G);
    Wire.write(ctrl7_g_val); 
    Wire.endTransmission();
    
    Serial.println("Filtry sprzetowe zostaly skonfigurowane.");
}

void Kalman_init(float initial_angle) {
    kalman_angle = initial_angle;
    kalman_bias = 0;
    kalman_P[0][0] = 1;
    kalman_P[0][1] = 0;
    kalman_P[1][0] = 0;
    kalman_P[1][1] = 1;
}

// Zmodyfikowana funkcja Kalman_update, przyjmujaca dynamiczne Q_angle i Q_bias
void Kalman_update(float newAngle, float newRate, float dt, float current_Q_angle, float current_Q_bias) {
    kalman_angle += dt * (newRate - kalman_bias);
    // ZMIANA: Uzycie dynamicznych Q_angle i Q_bias
    kalman_P[0][0] += dt * (dt * kalman_P[1][1] - kalman_P[0][1] - kalman_P[1][0] + current_Q_angle);
    kalman_P[0][1] -= dt * kalman_P[1][1];
    kalman_P[1][0] -= dt * kalman_P[1][1];
    kalman_P[1][1] += dt * current_Q_bias;
    float y = newAngle - kalman_angle;
    float S = kalman_P[0][0] + R_measure; // R_measure jest zmienna statyczna
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

static void saveCalibration(float roll, float pitch) {
    Serial.println("Rozpoczynam zapis kalibracji do pamieci QSPI...");
    calibration_data_t cal_data = {
        .roll = roll,
        .pitch = pitch
    };
    saveConfig(CALIBRATION_BLOCK, &cal_data, sizeof(cal_data));
}

static bool loadCalibration() {
    Serial.println("Rozpoczynam ladowanie kalibracji z pamieci QSPI...");
    calibration_data_t loaded_data;
    calibration_data_t factory_data = {
        .roll = default_roll_offset,
        .pitch = default_pitch_offset
    };

    if (loadConfig(CALIBRATION_BLOCK, &loaded_data, &factory_data, sizeof(loaded_data))) {
        roll_offset = loaded_data.roll;
        pitch_offset = loaded_data.pitch;
        return true;
    }
    return false;
}

bool LoadImuCalibration(void) {
    if (loadCalibration()) {
        Serial.println("Odczytano dane kalibracyjne z pamieci Flash.");
        Serial.print("Offset Roll: ");
        Serial.println(roll_offset);
        Serial.print("Offset Pitch: ");
        Serial.println(pitch_offset);
        float initial_accel_x = myIMU.readFloatAccelX();
        float initial_accel_z = myIMU.readFloatAccelZ();
        float initial_angle = atan2(initial_accel_x, initial_accel_z) * 180.0 / PI;
        Kalman_init(initial_angle);
        return true;
    } else {
        Serial.println("Brak danych kalibracyjnych w pamieci. Wymagana kalibracja!");
        return false;
    }
}

void setCallIMUasZero(void) {
    Serial.println(F("Ustawiam biezaca pozycje jako zero. Trzymaj uklad nieruchomo."));
    int cal_sample_count = 0;
    float cal_accel_x_sum = 0.0;
    float cal_accel_y_sum = 0.0;
    float cal_accel_z_sum = 0.0;

    while (cal_sample_count < 200) {
        float current_accel_x = myIMU.readFloatAccelX();
        float current_accel_y = myIMU.readFloatAccelY();
        float current_accel_z = myIMU.readFloatAccelZ();

        if (!isnan(current_accel_x) && !isnan(current_accel_y) && !isnan(current_accel_z)) {
            cal_accel_x_sum += current_accel_x;
            cal_accel_y_sum += current_accel_y;
            cal_accel_z_sum += current_accel_z;
            cal_sample_count++;
        }
        delay(10);
    }

    if (cal_sample_count > 0) {
        float avg_accel_y = cal_accel_y_sum / cal_sample_count;
        float avg_accel_z = cal_accel_z_sum / cal_sample_count;
        float avg_accel_x = cal_accel_x_sum / cal_sample_count;

        roll_offset = 0.0; 
        pitch_offset = atan2(avg_accel_x, avg_accel_z) * 180.0 / PI;

        Serial.println("Nowa pozycja 'zero' ustawiona.");
    } else {
        Serial.println("Nie udalo sie odczytac danych z IMU. Nie mozna ustawic nowej pozycji 'zero'.");
    }
    Kalman_init(0.0);
}

bool imu_init() {
    Wire.begin();
    if (myIMU.begin() != 0) {
        Serial.println("Device error - LSM6DS3TR-C not found during init!");
        return false;
    } else {
        Serial.println("LSM6DS3TR-C zainicjalizowany pomyslnie!");
        initHWFilter();
        return true;
    }
}

bool Call_IMU_Calibration(int write) {
    Serial.println(F("Rozpoczeto kalibracje. Trzymaj uklad nieruchomo."));
    int cal_sample_count = 0;
    float cal_accel_x_sum = 0.0;
    float cal_accel_y_sum = 0.0;
    float cal_accel_z_sum = 0.0;

    while (cal_sample_count < 200) {
        float current_accel_x = myIMU.readFloatAccelX();
        float current_accel_y = myIMU.readFloatAccelY();
        float current_accel_z = myIMU.readFloatAccelZ();

        if (!isnan(current_accel_x) && !isnan(current_accel_y) && !isnan(current_accel_z)) {
            cal_accel_x_sum += current_accel_x;
            cal_accel_y_sum += current_accel_y;
            cal_accel_z_sum += current_accel_z;
            cal_sample_count++;
            if (cal_sample_count % 20 == 0) {
                Serial.print(".");
            }
        }
        delay(10);
    }
    Serial.println();

    if (cal_sample_count > 0) {
        float avg_accel_y = cal_accel_y_sum / cal_sample_count;
        float avg_accel_z = cal_accel_z_sum / cal_sample_count;
        float avg_accel_x = cal_accel_x_sum / cal_sample_count;

        roll_offset = 0.0;
        pitch_offset = atan2(avg_accel_x, avg_accel_z) * 180.0 / PI;
        if (write) {
            saveCalibration(roll_offset, pitch_offset);
        }
        Serial.println("Kalibracja IMU zakonczona. Biezaca pozycja ustawiona jako zero.");
        Serial.print("Nowo obliczony Offset Roll: ");
        Serial.println(roll_offset);
        Serial.print("Nowo obliczony Offset Pitch: ");
        Serial.println(pitch_offset);

        return true;
    } else {
        Serial.println("Kalibracja IMU nie powiodla sie.");
        roll_offset = 0.0;
        pitch_offset = 0.0;
        return false;
    }
}

bool readIMUData(IMUData* data) {
    if (!data) {
        return false;
    }
    static int refreshTimer = 0;
    static float lastData = 0;

    // Twoja modyfikacja, ktora kontroluje czestotliwosc aktualizacji danych na LCD.
    //if (++refreshTimer % 3 == 0) {
      data->X=X/probe;
      data->Y=Y/probe;
      data->Z=Z/probe;
      X=0;
      Y=0;
      Z=0;
        int temp_probe = probe;
        probe = 0;

        data->tilt = kalman_angle - pitch_offset;
        if (data->tilt > 99.0) {
            data->tilt = 99.0;
        }
        lastData = data->tilt;
        data->kalmanAngle = kalman_angle;
        data->samplesCollected = temp_probe;

        Serial.print("Korekta Tilt (finalny wynik): ");
        Serial.println(data->tilt);

        char temp[30];
        sprintf(temp, "Gyro probe %d\n", temp_probe);
        Serial.println(temp);
    //}
    data->tilt = lastData; // Kopiowanie ostatniej, ustabilizowanej wartosci.
    
    return true;
}

void CollectImu(void) {
    // ======================================================================
    // 1. DYNAMICZNE OBLICZANIE dt
    // ======================================================================
    static unsigned long last_collect_time = 0;
    unsigned long current_time = micros();

    if (last_collect_time == 0) {
        last_collect_time = current_time;
        // Inicjalizacja statycznych filtrow LPF
        filtered_accel_x = myIMU.readFloatAccelX();
        filtered_accel_z = myIMU.readFloatAccelZ();
        return;
    }
    
    // Obliczenie dt w sekundach.
    float dt = (current_time - last_collect_time) / 1000000.0f;
    last_collect_time = current_time;

    // Odczyt surowych danych
    float accel_x = myIMU.readFloatAccelX();
    float accel_y = myIMU.readFloatAccelY(); // Os Roll
    float accel_z = myIMU.readFloatAccelZ(); // Os Pitch (Przyspieszenie/Hamowanie)
    float gyro_y = myIMU.readFloatGyroY();   // Os obrotu dla Pitch
    X+=accel_x;
    Y+=accel_y;
    Z+=accel_z;

    // Sprawdzenie poprawnosci odczytow
    if (isnan(accel_x) || isnan(accel_z) || isnan(gyro_y)) {
        return;
    }

    // ======================================================================
    // 2. FILTR LPF (maksymalne wygladzenie wibracji)
    // ======================================================================
    // Ekstremalnie niska wartosc ALPHA powoduje CELOWE opoznienie i usuniecie wibracji.
    filtered_accel_x = ACCEL_LPF_ALPHA * accel_x + (1.0f - ACCEL_LPF_ALPHA) * filtered_accel_x;
    filtered_accel_z = ACCEL_LPF_ALPHA * accel_z + (1.0f - ACCEL_LPF_ALPHA) * filtered_accel_z;

    // Obliczanie kata na podstawie odfiltrowanych danych (konfiguracja X/Z jest prawidlowa dla Twojej plytki)
    float accel_angle = atan2(filtered_accel_x, filtered_accel_z) * 180.0 / PI;
    
    // ======================================================================
    // 3. LOGIKA ADAPTACYJNA (eliminacja bledu przyspieszania/hamowania i stabilizacja biasu)
    // ======================================================================
    
    // Suma kwadratow predkosci katowych (calkowity ruch obrotowy)
    float gyro_norm = sqrt(myIMU.readFloatGyroX() * myIMU.readFloatGyroX() + 
                            myIMU.readFloatGyroY() * myIMU.readFloatGyroY() + 
                            myIMU.readFloatGyroZ() * myIMU.readFloatGyroZ());
                            
    // Obliczanie normy (dlugosci wektora) przyspieszenia.
    float accel_norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    // KLUCZOWA ZMIANA LOGIKI: Przelaczanie R_measure I Q_bias
    float current_Q_bias;
    float current_Q_angle = Q_angle_base; // Q_angle jest stale

    // NOWY WARUNEK: Sprawdzenie, czy wykryto ruch obrotowy, przyspieszenie liniowe (Norma) 
    // LUB WYKRYTO ZNACZĄCE PRZYSPIESZENIE/HAMOWANIE W OSI Z (która u Ciebie odpowiada za ruch).
    if (gyro_norm > GYRO_THRESHOLD || abs(accel_norm - 1.0f) > ACCEL_TOLERANCE || abs(accel_z) > ACCEL_Z_THRESHOLD) {
        // TRYB RUCHU (Niski poziom zaufania do Accel, Wysoki szum biasu)
        // Wymuszone ignorowanie Accel, gdy auto przyspiesza.
        R_measure = R_measure_high;  
        current_Q_bias = Q_bias_high; 
    } else {
        // TRYB SPOCZYNKU (Wysoki poziom zaufania do Accel, Niski szum biasu)
        R_measure = R_measure_low;    
        current_Q_bias = Q_bias_low;  
    }

    // Aktualizacja filtru Kalmana z dynamicznym R_measure, Q_angle i Q_bias.
    Kalman_update(accel_angle, gyro_y, dt, current_Q_angle, current_Q_bias);
    probe++;
}