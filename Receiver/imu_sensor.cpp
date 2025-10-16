#include "imu_sensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "ConfigurationStorage.h"  // Używa funkcji saveConfig i loadConfig

#define CTRL8_XL 0x17
#define CTRL6_C 0x15
#define CTRL7_G 0x16
#define LSM6DS3_ADDR 0x6A
#define CALIBRATION_BLOCK 0

LSM6DS3 myIMU(I2C_MODE, LSM6DS3_ADDR);

static float kalman_angle = 0;
static float kalman_bias = 0;
static float kalman_P[2][2] = { { 0, 0 }, { 0, 0 } };

// ZMIENNE ZINICJALIZOWANE POPRAWNYMI STAŁYMI Z PLIKU .h

// Zmienna LPF do użycia w CollectImu()
static const float ACCEL_LPF_ALPHA = ACCEL_LPF_ALPHA_set;

// Zmienna, ktora jest dynamicznie zmieniana w CollectImu()
static float R_measure = R_measure_low_set; 

static const float Q_angle_base = Q_angle_set;
static const float Q_bias_low = Q_bias_low_set;
static const float Q_bias_high = Q_bias_high_set;

// Poniższe statyczne stałe muszą być użyte w logice adaptacyjnej (CollectImu)
static const float ACCEL_TOLERANCE = ACCEL_TOLERANCE_set;
static const float R_measure_high = R_measure_high_set;
static const float R_measure_low = R_measure_low_set;
static const float GYRO_THRESHOLD = GYRO_THRESHOLD_set;

// Te zmienne są używane w funkcjach get i CollectImu()
static float pitch_offset = 0;
static float accel_norm_debug = 0;
static bool movment = false;
static float X=0, Y=0, Z=0;
static int probe = 0;
static float filtered_accel_x = 0;
static float filtered_accel_z = 0;

// ======================================================================
// ZMIENNE KALIBRACYJNE (NOWA STRUKTURA)
// ======================================================================
static float roll_offset = 0.0;
static float g_x_offset = 0.0;  // NOWA: Statyczna wartość grawitacji X (PIONOWA)
static float g_z_offset = 0.0;  // NOWA: Statyczna wartość grawitacji Z (KIERUNEK JAZDY)

static const float default_roll_offset = 0.0;
static const float default_pitch_offset = 0.0;
static const float default_g_x_offset = 1.0f;
static const float default_g_z_offset = 0.0f;


// ======================================================================
// FUNKCJE POMOCNICZE (BEZ ZMIAN)
// ======================================================================
bool getGyroState(float* rawGyro) {
  *rawGyro = kalman_bias;
  return movment;
}

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

void Kalman_update(float newAngle, float newRate, float dt, float current_Q_angle, float current_Q_bias) {
  kalman_angle += dt * (newRate - kalman_bias);
  kalman_P[0][0] += dt * (dt * kalman_P[1][1] - kalman_P[0][1] - kalman_P[1][0] + current_Q_angle);
  kalman_P[0][1] -= dt * kalman_P[1][1];
  kalman_P[1][0] -= dt * kalman_P[1][1];
  kalman_P[1][1] += dt * current_Q_bias;
  float y = newAngle - kalman_angle;
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

// ======================================================================
// OBSŁUGA PAMIĘCI FLASH (ZMODYFIKOWANE)
// ======================================================================

// Nowa funkcja zapisu: UWZGLĘDNIENIE G_X_OFFSET i G_Z_OFFSET
static void saveCalibration(float roll, float pitch, float g_x, float g_z) {
  Serial.println("Rozpoczynam zapis kalibracji do pamieci QSPI...");
  calibration_data_t cal_data = {
    .roll = roll,
    .pitch = pitch,
    .g_x_offset = g_x,
    .g_z_offset = g_z
  };
  saveConfig(CALIBRATION_BLOCK, &cal_data, sizeof(cal_data));
}

// Zmodyfikowana funkcja ładowania: UWZGLĘDNIENIE G_X_OFFSET i G_Z_OFFSET
static bool loadCalibration() {
  Serial.println("Rozpoczynam ladowanie kalibracji z pamieci QSPI...");
  calibration_data_t loaded_data;
  calibration_data_t factory_data = {
    .roll = default_roll_offset,
    .pitch = default_pitch_offset,
    .g_x_offset = default_g_x_offset,
    .g_z_offset = default_g_z_offset
  };

  if (loadConfig(CALIBRATION_BLOCK, &loaded_data, &factory_data, sizeof(loaded_data))) {
    roll_offset = loaded_data.roll;
    pitch_offset = loaded_data.pitch;
    g_x_offset = loaded_data.g_x_offset;
    g_z_offset = loaded_data.g_z_offset;
    return true;
  }
  // Użycie domyślnych wartości w przypadku niepowodzenia
  g_x_offset = default_g_x_offset;
  g_z_offset = default_g_z_offset;
  return false;
}

bool LoadImuCalibration(void) {
  if (loadCalibration()) {
    Serial.println("Odczytano dane kalibracyjne z pamieci Flash.");
    Serial.print("Offset Roll: ");
    Serial.println(roll_offset);
    Serial.print("Offset Pitch: ");
    Serial.println(pitch_offset);
    Serial.print("Offset G_X: ");
    Serial.println(g_x_offset);
    Serial.print("Offset G_Z: ");
    Serial.println(g_z_offset);
    Kalman_init(pitch_offset);
    return true;
  } else {
    Serial.println("Brak danych kalibracyjnych w pamieci. Uzycie wartosci domyslnych.");
    Kalman_init(0.0);
    return false;
  }
}

// ======================================================================
// FUNKCJE KALIBRACYJNE (ZMODYFIKOWANE)
// ======================================================================

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
    float avg_accel_x = cal_accel_x_sum / cal_sample_count;  // Statyczny Gx
    float avg_accel_z = cal_accel_z_sum / cal_sample_count;  // Statyczny Gz

    roll_offset = 0.0;
    // Kąt Pitch liczony z wektora (Az, Ax)
    pitch_offset = atan2(avg_accel_z, avg_accel_x) * 180.0 / PI;

    g_x_offset = avg_accel_x;  // ZAPIS ŚREDNIEJ WARTOSCI GRAWITACJI X
    g_z_offset = avg_accel_z;  // ZAPIS ŚREDNIEJ WARTOSCI GRAWITACJI Z

    Serial.println("Nowa pozycja 'zero' ustawiona.");
  } else {
    Serial.println("Nie udalo sie odczytac danych z IMU. Nie mozna ustawic nowej pozycji 'zero'.");
  }
  Kalman_init(0.0);
}

// Zmodyfikowana funkcja kalibracji: ZAPIS OBU WARTOSCI (G_X i G_Z)
bool Call_IMU_Calibration(int write) {
  Serial.println(F("Rozpoczeto kalibracje. Trzymaj uklad nieruchomo."));
  int cal_sample_count = 0;
  float cal_accel_x_sum = 0.0;
  float cal_accel_y_sum = 0.0;
  float cal_accel_z_sum = 0.0;

  while (cal_sample_count < 200) {
    // ... (zbieranie probek) ...
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
    float avg_accel_x = cal_accel_x_sum / cal_sample_count;  // Statyczny Gx
    float avg_accel_z = cal_accel_z_sum / cal_sample_count;  // Statyczny Gz

    roll_offset = 0.0;
    pitch_offset = atan2(avg_accel_z, avg_accel_x) * 180.0 / PI;

    g_x_offset = avg_accel_x;
    g_z_offset = avg_accel_z;

    if (write) {
      saveCalibration(roll_offset, pitch_offset, g_x_offset, g_z_offset);  // Zapis obu offsetow
    }
    Serial.println("Kalibracja IMU zakonczona. Biezaca pozycja ustawiona jako zero.");
    Serial.print("Nowo obliczony Offset Roll: ");
    Serial.println(roll_offset);
    Serial.print("Nowo obliczony Offset Pitch: ");
    Serial.println(pitch_offset);
    Serial.print("Nowo obliczony G_X Offset: ");
    Serial.println(g_x_offset);
    Serial.print("Nowo obliczony G_Z Offset: ");
    Serial.println(g_z_offset);

    return true;
  } else {
    Serial.println("Kalibracja IMU nie powiodla sie.");
    roll_offset = 0.0;
    pitch_offset = 0.0;
    g_x_offset = default_g_x_offset;
    g_z_offset = default_g_z_offset;
    return false;
  }
}

// ... (funkcja imu_init - BEZ ZMIAN) ...
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


// ... (funkcja readIMUData - BEZ ZMIAN) ...
bool readIMUData(IMUData* data) {
  if (!data) {
    return false;
  }
  static int refreshTimer = 0;
  static float lastData = 0;

  // Twoja modyfikacja, ktora kontroluje czestotliwosc aktualizacji danych.
  //if (++refreshTimer % 3 == 0) {
  data->X = X / probe;
  data->Y = Y / probe;
  data->Z = Z / probe;
  X = 0;
  Y = 0;
  Z = 0;
  int temp_probe = probe;
  probe = 0;

  data->tilt = (kalman_angle * (-1));  // - pitch_offset;
  if (data->tilt > 60.0) {
    data->tilt = 60.0;
  } else {
    if (data->tilt < -60.0) {
      data->tilt = -60.0;
    }
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
  data->tilt = lastData;

  return true;
}


// KLUCZOWA ZMODYFIKOWANA FUNKCJA
// ======================================================================
// 4. GLOWNA PETLA Z LOGIKA ADAPTACYJNA (CollectImu)
// ======================================================================

void CollectImu(void) {
  static unsigned long last_collect_time = 0;
  unsigned long current_time = micros();

  if (last_collect_time == 0) {
    last_collect_time = current_time;
    // Inicjalizacja statycznych filtrow LPF
    filtered_accel_x = myIMU.readFloatAccelX();
    filtered_accel_z = myIMU.readFloatAccelZ();
    return;
  }

  float dt = (current_time - last_collect_time) / 1000000.0f;
  last_collect_time = current_time;

  // Odczyt surowych danych
  float accel_x = myIMU.readFloatAccelX();
  float accel_y = myIMU.readFloatAccelY();
  float accel_z = myIMU.readFloatAccelZ();
  float gyro_y = myIMU.readFloatGyroY();

  // SUMOWANIE DANYCH DIAGNOSTYCZNYCH
  X += accel_x;
  Y += accel_y;
  Z += accel_z;

  if (isnan(accel_x) || isnan(accel_z) || isnan(gyro_y)) {
    return;
  }

  // ======================================================================
  // 2. FILTR LPF
  // ======================================================================
  // Używamy oryginalnych, przefiltrowanych danych do obliczenia kąta ACCEL
  filtered_accel_x = ACCEL_LPF_ALPHA * accel_x + (1.0f - ACCEL_LPF_ALPHA) * filtered_accel_x;
  filtered_accel_z = ACCEL_LPF_ALPHA * accel_z + (1.0f - ACCEL_LPF_ALPHA) * filtered_accel_z;

  // Kąt z Akcelerometru (surowy)
  float accel_angle_raw = atan2(filtered_accel_z, filtered_accel_x) * 180.0 / PI;

  // ======================================================================
  // 3. LOGIKA ADAPTACYJNA (Przełączanie trybu)
  // ======================================================================

  // Obliczanie normy (do wykrywania ruchu obrotowego)
  float gyro_norm = sqrt(myIMU.readFloatGyroX() * myIMU.readFloatGyroX() + myIMU.readFloatGyroY() * myIMU.readFloatGyroY() + myIMU.readFloatGyroZ() * myIMU.readFloatGyroZ());
  // Obliczanie normy (do wykrywania przyspieszenia liniowego)
  float accel_norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
  // Wskaźnik przyspieszenia liniowego (Norma != 1G)
  bool is_linear_accel = abs(accel_norm - 1.0f) > ACCEL_TOLERANCE;
  // Wskaźnik ruchu obrotowego
  bool is_gyro_movement = gyro_norm > GYRO_THRESHOLD;

  #if 0     //ustawion dla debug
  // -----------------------------------------------------------------
  // DEBUGOWANIE W KONSOLI (POPRAWIONE)
  // -----------------------------------------------------------------
    // ZAPISZ NORME AKCELEROMETRU DO ZMIENNEJ STATYCZNEJ DLA DEBUGOWANIA
  accel_norm_debug = accel_norm;  // <-- Używamy statycznej zmiennej
  Serial.print("Norma: ");
  Serial.print(accel_norm, 4);  // Norma przyspieszenia, 4 miejsca po przecinku

  Serial.print(" | is_linear_accel: ");
  Serial.print(is_linear_accel ? "TRUE (Ruch)" : "FALSE (Postój)");

  Serial.print(" | is_gyro_movement: ");
  Serial.print(is_gyro_movement ? "TRUE (Ruch)" : "FALSE (Postój)");

  Serial.print(" | Tryb MOVMENT: ");
  Serial.println(movment ? "DYNAMICZNY (Czerwony)" : "KOREKCJA (Zielony)");

  Serial.print(" | GyroNorm: ");
  Serial.print(gyro_norm, 4);  // Wydrukuj normę żyroskopu
  // -----------------------------------------------------------------
  #endif

  float current_Q_bias;
  float current_Q_angle = Q_angle_base;

  // PRZELACZANIE TRYBU ADAPTACYJNEGO
  // Polegamy tylko na Normie Accel i Normie Gyro!
  if (is_linear_accel || is_gyro_movement) {
    // TRYB DYNAMIKA (Ruch) - Ufamy żyroskopowi (wysokie R_measure)
    R_measure = R_measure_high;
    current_Q_bias = Q_bias_high;
    movment = true;

  } else {
    // TRYB KOREKCJA (Postój/Stała Prędkość) - Ufamy akcelerometrowi (niskie R_measure)
    R_measure = R_measure_low;
    current_Q_bias = Q_bias_low;
    movment = false;
  }


  // -------------------------------------------------------------
  // KOREKTA KATA ACCEL (ZEROWANIE offsetu montazu)
  // -------------------------------------------------------------
  // Wartość pitch_offset (z kalibracji) jest odejmowana, aby skompensować montaż.
  float accel_angle_corrected = accel_angle_raw - pitch_offset;

  // 4. AKTUALIZACJA FILTRU KALMANA
  Kalman_update(accel_angle_corrected, gyro_y, dt, current_Q_angle, current_Q_bias);
  probe++;
}