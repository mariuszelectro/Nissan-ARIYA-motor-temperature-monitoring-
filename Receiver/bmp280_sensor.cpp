#include "bmp280_sensor.h"
#include <Wire.h> 

// Utworzenie obiektu czujnika BMP280
// Domyślny adres I2C dla BMP280 to zazwyczaj 0x76 lub 0x77
Adafruit_BMP280 bmp;

// Globalna zmienna do przechowywania ciśnienia na poziomie morza w hPa
// Używana do obliczania wysokości nad poziomem morza.
static float seaLevelhPa = 1013.25;

/**
 * @brief Ustawia ciśnienie na poziomie morza na podstawie aktualnego ciśnienia i zadanej wysokości.
 * @param meters Wysokość bezwzględna w metrach (np. z GPS).
 */
void setAbsoluteHighest(int meters) {
    float currentPressurePa = bmp.readPressure();
    if (isnan(currentPressurePa)) {
        Serial.println(F("Nie udalo sie odczytac cisnienia do kalibracji wysokosci."));
        return;
    }
    // Obliczanie ciśnienia na poziomie morza w hPa z Pa
    seaLevelhPa = bmp.seaLevelForAltitude(meters, currentPressurePa / 100.0);
    Serial.print(F("Ustawiono wysokosc bezwzgledna na: "));
    Serial.print(meters);
    Serial.print(F(" m. Cisnienie na poziomie morza ustawione na: "));
    Serial.print(seaLevelhPa);
    Serial.println(F(" hPa"));
}

/**
 * @brief Inicjalizuje czujnik BMP280.
 * @return true jesli inicjalizacja powiodla sie, false w przeciwnym wypadku.
 */
bool initBmp280() {
    Wire.begin(); // Uruchomienie magistrali I2C
    bool status = bmp.begin(0x76); // Spróbuj z adresem 0x76

    if (!status) {
        // Jeśli 0x76 nie zadziała, spróbuj 0x77 (dla niektórych modułów)
        status = bmp.begin(0x77);
        if (!status) {
            Serial.println(F("Nie znaleziono czujnika BMP280! Sprawdz okablowanie lub adres I2C (0x76/0x77)."));
            return false;
        }
    }

    // Opcjonalne: Ustawienia próbkowania dla BMP280 (możesz je dostosować)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Tryb pracy
                    Adafruit_BMP280::SAMPLING_X2,      // Próbkowanie temperatury
                    Adafruit_BMP280::SAMPLING_X16,     // Próbkowanie cisnienia
                    Adafruit_BMP280::FILTER_X16,       // Filtr IIR
                    Adafruit_BMP280::STANDBY_MS_500);  // Czas czuwania
    
    Serial.println(F("Czujnik BMP280 zainicjowany pomyslnie."));
    return true;
}

/**
 * @brief Odczytuje dane z czujnika BMP280 i zapisuje je do struktury.
 * @param data Wskaznik do struktury Bmp280Data, ktora zostanie wypelniona odczytanymi danymi.
 * @return true jesli odczyt powiodl sie, false w przeciwnym wypadku.
 */
bool readBmp280Data(Bmp280Data* data) {
    if (!data) { // Sprawdzenie, czy wskaznik jest poprawny
        Serial.println(F("Blad: wskaznik do Bmp280Data jest NULL."));
        return false;
    }

    float temp = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude(seaLevelhPa);

    // Sprawdzenie, czy odczyty są poprawne (nie NaN)
    if (isnan(temp) || isnan(pressure) || isnan(altitude)) {
        Serial.println(F("Nie udalo sie odczytac danych z czujnika BMP280!"));
        return false;
    }

    data->temperature = temp;
    data->pressure = pressure;
    data->altitude = altitude; // Zapisujemy obliczoną wysokość
    return true;
}