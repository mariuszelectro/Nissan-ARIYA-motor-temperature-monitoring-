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


static float Q_angle =  Q_angle_set;
static float Q_bias =Q_bias_set;
static float R_measure = R_measure_set;

static const float R_measure_high = R_measure_high_set;
static const float R_measure_low = R_measure_low_set;
static const float GYRO_THRESHOLD = GYRO_THRESHOLD_set;
static const float ACCEL_TOLERANCE = ACCEL_TOLERANCE_set; 
static const float ACCEL_LPF_ALPHA =  ACCEL_LPF_ALPHA_set; 

static int probe = 0;

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

void Kalman_update(float newAngle, float newRate, float dt) {
  kalman_angle += dt * (newRate - kalman_bias);
  kalman_P[0][0] += dt * (dt * kalman_P[1][1] - kalman_P[0][1] - kalman_P[1][0] + Q_angle);
  kalman_P[0][1] -= dt * kalman_P[1][1];
  kalman_P[1][0] -= dt * kalman_P[1][1];
  kalman_P[1][1] += dt * Q_bias;
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

  // Twoja modyfikacja, która kontroluje częstotliwość aktualizacji danych na LCD.
  //if (++refreshTimer % 3 == 0) {
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
  data->tilt = lastData; // Kopiowanie ostatniej, ustabilizowanej wartości.
  return true;
}

void CollectImu(void) {
  const float dt = 0.02;

  // Odczyt surowych danych
  float accel_x = myIMU.readFloatAccelX();
  float accel_y = myIMU.readFloatAccelY();
  float accel_z = myIMU.readFloatAccelZ();
  float gyro_y = myIMU.readFloatGyroY(); 

  if (isnan(accel_x) || isnan(accel_z) || isnan(gyro_y)) {
    return;
  }

  // Filtr dolnoprzepustowy (LPF) na danych z akcelerometru
  filtered_accel_x = ACCEL_LPF_ALPHA * accel_x + (1 - ACCEL_LPF_ALPHA) * filtered_accel_x;
  filtered_accel_z = ACCEL_LPF_ALPHA * accel_z + (1 - ACCEL_LPF_ALPHA) * filtered_accel_z;

  // Obliczanie kąta na podstawie odfiltrowanych danych
  float accel_angle = atan2(filtered_accel_x, filtered_accel_z) * 180.0 / PI;
  
  // Obliczanie normy (długości wektora) prędkości obrotowej i przyspieszenia
  float gyro_norm = sqrt(myIMU.readFloatGyroX() * myIMU.readFloatGyroX() +
                         myIMU.readFloatGyroY() * myIMU.readFloatGyroY() +
                         myIMU.readFloatGyroZ() * myIMU.readFloatGyroZ());
  
  float accel_norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

  // Adaptacyjne dostosowanie R_measure
  if (gyro_norm > GYRO_THRESHOLD || abs(accel_norm - 1.0) > ACCEL_TOLERANCE) {
    R_measure = R_measure_high;
  } else {
    R_measure = R_measure_low;
  }

  Kalman_update(accel_angle, gyro_y, dt);
  probe++;
}