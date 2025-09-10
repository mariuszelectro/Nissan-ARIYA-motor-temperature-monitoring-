#include "ble_control.h"
#include "BLE_calibration.h"
#include <cstdio>
#include <cstring>
#include <Arduino.h>

// Deklaracja zewnętrznej struktury
extern struct DisplayData* globalDisplayData;

// Wewnętrzna zmienna do zatrzymania skanowania
static bool beaconFound = false;

void checkAndCalibrateWithBeacon(const uint8_t* p_data, uint8_t len, int8_t rssi) {

  // Zatrzymanie poszukiwań, jeśli licznik się wyczerpał lub beacon został już znaleziony
  if (globalDisplayData->MainCalibrate.CalibrationConuter <= 0 || beaconFound) {
    return;
  }

  uint8_t pos = 0;
  while (pos < len) {
    uint8_t sectionLen = p_data[pos];
    if (sectionLen == 0 || pos + sectionLen + 1 > len) {
      break;
    }
    uint8_t sectionType = p_data[pos + 1];
    const uint8_t* section_data_ptr = &p_data[pos + 2];
    int current_data_len = sectionLen - 1;

    // Sprawdzamy, czy to są dane producenta (typ 0xFF)
    if (sectionType == 0xFF) {
      // Szukamy prefiksu "BAT:" w danych producenta
      const char* batPrefix = "BAT:";
      // Sprawdzamy, czy dane są wystarczająco długie i czy zawierają nasz prefix
      // Prefiks "BAT:" jest w buforze danych producenta, po 2 bajtach ID producenta.
      if (current_data_len >= (2 + strlen(batPrefix) + 2 + 4) && memcmp(section_data_ptr + 2, batPrefix, strlen(batPrefix)) == 0) {
        
        // Znaleziono beacon!
        beaconFound = true;
        
        // Odczytujemy napięcie
        uint16_t voltage_mV;
        memcpy(&voltage_mV, section_data_ptr + 2 + strlen(batPrefix), sizeof(voltage_mV));
        float voltage_V = (float)voltage_mV / 1000.0f;

        // Odczytujemy licznik
        int count;
        memcpy(&count, section_data_ptr + 2 + strlen(batPrefix) + sizeof(voltage_mV), sizeof(count));

        // Formatujemy licznik na dni, godziny, minuty i sekundy
        int totalSeconds = count;
        int days = totalSeconds / 86400; // SecYear
        totalSeconds %= 86400;
        int hours = totalSeconds / 3600;
        totalSeconds %= 3600;
        int minutes = totalSeconds / 60;
        int seconds = totalSeconds % 60;
        
        // Zapisujemy sformatowany tekst do bufora z uwzględnieniem ograniczenia 32 bajtów
        // Teraz z wartością RSSI
        snprintf(globalDisplayData->MainCalibrate.TextValue, sizeof(globalDisplayData->MainCalibrate.TextValue), "rssi:%d %.3fV %dD%dh:%dm:%d", rssi, voltage_V, days, hours, minutes, seconds);

        // Ustawiamy flagę kalibracji na TRUE
        globalDisplayData->MainCalibrate.RealCalibrate = true;
        
        // Wypisujemy informacje debugowe, aby potwierdzić działanie
        Serial.println("INFO: Znaleziono beacon kalibracyjny!");
        Serial.printf("INFO: Napiecie: %.3fV\n", voltage_V);
        Serial.printf("INFO: Licznik: %dD %dh:%dm:%ds\n", days, hours, minutes, seconds);
        Serial.printf("INFO: RSSI: %d\n", rssi);

        break; // Koniec szukania, znaleziono i przetworzono dane
      }
    }
    pos += sectionLen + 1;
  }
}