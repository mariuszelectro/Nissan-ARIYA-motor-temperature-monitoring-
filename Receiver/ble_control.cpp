#include "ble_control.h"
#include "Config.h"       // Zawiera BUZZER_PIN, BEACON_DATA_TIMEOUT_MS, BEACON_ALIVE_COUNTER_TIMEOUT_MS
#include "LCD_Display.h"  // Aby DisplayData bylo widoczne dla ble_task (mimo ze nie modyfikujemy 'source', potrzebujemy definicji struktury)
#include <bluefruit.h>
#include <cstdio>     // Dla sscanf
#include <cstdlib>    // Dla atof, atoi
#include <cstring>    // Dla strcmp
#include <Arduino.h>  // Potrzebne dla tone(), noTone(), millis()
#include <algorithm>  // Dla std::max
#include <cmath>      // Dla abs() i roundf()
#include "BLE_calibration.h"

// Flaga, która zapobiega wielokrotnemu drukowaniu informacji o beaconie
volatile bool foundBeaconFlag = false;
DisplayData* globalDisplayData = nullptr;        //zadeklarowne na potrzyb dostępu dla innych funkcji z poza tego pliku 
// Lokalna struktura do przechowywania danych z beacona
struct BeaconData {
  float temp_analog;
  float temp_digital;
  char alarm_status;
  char fan_status;
  int sensor_alive_counter;
  int8_t rssi_value;
  unsigned long timestamp;  // Dodano znacznik czasu, aby wiedziec, kiedy ostatnio odebrano dane
};

// Statyczna globalna zmienna do przechowywania ostatnich sparsowanych danych z beacona
static BeaconData lastParsedBeaconData={.temp_analog=-1,.temp_digital=-1};
static unsigned long lastBeaconDataUpdateTime = 0;  // Znacznik czasu ostatniej aktualizacji beacona

// Statyczne zmienne globalne do akumulowania temperatury do uśredniania
static float accumulatedTempFromBeacons = 0.0;
static int accumulatedCountFromBeacons = 0;
static char source=' ';

// === Nowe zmienne do sledzenia licznika zycia sensora ===
static int lastKnownAliveCounter = -1;                // Ostatnia znana wartosc licznika (-1 oznacza nieznana/poczatkowa)
static unsigned long lastAliveCounterChangeTime = 0;  // Czas ostatniej zmiany licznika
// =======================================================

// === Nowe zmienne do akumulowania RSSI ===
static float accumulatedRssiFromBeacons = 0.0;
static int accumulatedRssiCount = 0;
// =========================================

// Funkcja do parsowania pakietow reklamowych
// Teraz funkcja przyjmuje tylko surowe dane BLE i RSSI
void parseAndPrintAdvData(const uint8_t* data, uint8_t len, int8_t rssi) {
  uint8_t pos = 0;
  while (pos < len) {
    uint8_t sectionLen = data[pos];
    if (sectionLen == 0 || pos + sectionLen + 1 > len) {
      break;
    }

    // Pomijamy naglowek sekcji (Type + Data)
    const char* data_str = (const char*)&data[pos + 2];
    int current_data_len = sectionLen - 1;

    // Sprawdzamy czy dane reklamowe pasuja do nowego wzorca "Axxx.x Cxxx.x iiL"
    // Minimalna dlugosc stringa: A0.0 C0.0 NN0 (14 znakow)
    if (current_data_len >= 14 && data_str[0] == 'A') {

      // Tymczasowe zmienne do parsowania
      char temp_analog_str[6];  // np. "25.3" lub "---.-" (5 znakow + null)
      char temp_digital_str[6];
      char alarm_char;
      char fan_char;
      int counter_int;

      // Uzywamy sscanf do parsowania wzorca "A%5s C%5s %c%c%1d"
      int parsed_count = sscanf(data_str, "A%5s C%5s %c%c%1d",
                                temp_analog_str, temp_digital_str,
                                &alarm_char, &fan_char, &counter_int);

      // Sprawdzamy, czy parsowanie bylo udane (oczekujemy 5 odczytanych elementow)
      if (parsed_count == 5) {
        // Jesli udalo sie sparsowac, to jest to nasz beacon
        if (!foundBeaconFlag) {
          foundBeaconFlag = true;
          Serial.println("=========================================");
          Serial.println("=== Znaleziono Twoj Beacon! =============");
          Serial.println("=== (Wykryto poprawny format danych) ====");
          Serial.println("=========================================");
          Serial.print("Adres: ");
          Serial.println(" (Adres MAC beacona)");
        }

        // Wypelnianie LOKALNEJ struktury BeaconData (dla debugowania i tymczasowego przechowywania)
        BeaconData currentBeaconDataLocal;         // Tymczasowa lokalna struktura
        currentBeaconDataLocal.rssi_value = rssi;  // RSSI z raportu skanera

        // Parsowanie temperatury analogowej
        if (strcmp(temp_analog_str, "---.-") == 0) {
          currentBeaconDataLocal.temp_analog = -1.0;
        } else {
          currentBeaconDataLocal.temp_analog = atof(temp_analog_str);
        }

        // Parsowanie temperatury cyfrowej
        if (strcmp(temp_digital_str, "---.-") == 0) {
          currentBeaconDataLocal.temp_digital = -1.0;
        } else {
          currentBeaconDataLocal.temp_digital = atof(temp_digital_str);
        }

        currentBeaconDataLocal.alarm_status = alarm_char;
        currentBeaconDataLocal.fan_status = fan_char;
        currentBeaconDataLocal.sensor_alive_counter = counter_int;
        currentBeaconDataLocal.timestamp = millis();  // Zapisz czas odebrania

        // === Logika wyboru temperatury i dodawania do akumulatora średniej ===
        float selectedTemp = -1.0;
        bool analogValid = (currentBeaconDataLocal.temp_analog != -1.0);
        bool digitalValid = (currentBeaconDataLocal.temp_digital != -1.0);

        /*if (analogValid && digitalValid) {
          selectedTemp = std::max(currentBeaconDataLocal.temp_analog, currentBeaconDataLocal.temp_digital);
        } else if (analogValid) {
          selectedTemp = currentBeaconDataLocal.temp_analog;
        } else if (digitalValid) {
          selectedTemp = currentBeaconDataLocal.temp_digital;
        }*/
        // === Logika wyboru temperatury i dodawania do akumulatora średniej ===        

                if (analogValid && digitalValid) {
                    if (currentBeaconDataLocal.temp_analog > currentBeaconDataLocal.temp_digital) {
                        selectedTemp = currentBeaconDataLocal.temp_analog;
                        source = 'A'; // Wybrano A, bo jest większa
                    } else {
                        selectedTemp = currentBeaconDataLocal.temp_digital;
                        source = 'C'; // Wybrano C, bo jest większa lub równa
                    }
                } else if (analogValid) {
                    selectedTemp = currentBeaconDataLocal.temp_analog;
                    source = 'A'; // Wybrano A, bo tylko ona jest poprawna
                } else if (digitalValid) {
                    selectedTemp = currentBeaconDataLocal.temp_digital;
                    source = 'C'; // Wybrano C, bo tylko ona jest poprawna
                }
                
                if (selectedTemp != -1.0) { // Tylko jeśli wybrano prawidłową temperaturę
                    // Zapisujemy wybraną literę do globalnej struktury
                

                    // Akumulacja do uśredniania
                    accumulatedTempFromBeacons += selectedTemp;
                    accumulatedCountFromBeacons++;
                }
  
        if (selectedTemp != -1.0) {  // Tylko jeśli wybrano prawidłową temperaturę
          accumulatedTempFromBeacons += selectedTemp;
          accumulatedCountFromBeacons++;
          // lastBeaconDataReceivedTime jest aktualizowany raz w ble_task na podstawie ostatniego aktywnego beacona
        }
        // ===============================================================

        // === Akumulacja RSSI ===
        // RSSI jest zazwyczaj ujemne, wiec bierzemy wartosc absolutna
        accumulatedRssiFromBeacons += abs(rssi);
        accumulatedRssiCount++;
        // ========================

        // === Aktualizacja stanu licznika zycia sensora ===
        if (counter_int != lastKnownAliveCounter) {
          lastKnownAliveCounter = counter_int;
          lastAliveCounterChangeTime = millis();  // Licznik zmienil sie - resetuj czas
        }
        // ===============================================
        //tone(BUZZER_PIN, 1000); // Odtwarzaj dzwiek o czestotliwosci 1 KHz na pinie BUZZER_PIN
        // Wyswietlanie danych z LOKALNEJ struktury na monitorze szeregowym
        static int diagnose=0;
        if(diagnose++%10==0)
        {
        Serial.printf("Temp Sensor RSSI %d\n", currentBeaconDataLocal.rssi_value);
        /*Serial.print("RSSI: "); Serial.println(currentBeaconDataLocal.rssi_value);
                Serial.print("Temp Analog: "); Serial.print(currentBeaconDataLocal.temp_analog); Serial.println(" C");
                Serial.print("Temp Cyfrowa: "); Serial.print(currentBeaconDataLocal.temp_digital); Serial.println(" C");
                Serial.print("Alarm: "); Serial.println(currentBeaconDataLocal.alarm_status);
                Serial.print("Wentylator: "); Serial.println(currentBeaconDataLocal.fan_status);
                Serial.print("Licznik: "); Serial.println(currentBeaconDataLocal.sensor_alive_counter);
                Serial.println("=========================================");
                */
        // === Dodano: Sygnal dzwiekowy ===
        //noTone(BUZZER_PIN);    // Wylacz dzwiek
        // =============================
        }
        // Zapisz sparsowane dane do statycznej globalnej zmiennej
        lastParsedBeaconData = currentBeaconDataLocal;
        lastBeaconDataUpdateTime = currentBeaconDataLocal.timestamp;
      }
      return;  // Przetworzono sekcje, jesli pasuje do wzorca, wychodzimy
    }
    pos += sectionLen + 1;  // Przejdz do nastepnej sekcji
  }
}

// Funkcje callback dla BLE (polaczenie/rozlaczenie)
void connect_callback(uint16_t conn_handle) {
  (void)conn_handle;
  Serial.println("Urządzenie połączone.");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  Serial.println("Urządzenie rozłączone.");
}

// Funkcja callback dla skanera BLE
void scan_callback(ble_gap_evt_adv_report_t* report) {
  // Wywolujemy funkcje parsowania, ktora teraz przyjmuje tylko surowe dane i RSSI
  parseAndPrintAdvData(report->data.p_data, report->data.len, report->rssi);

  checkAndCalibrateWithBeacon(report->data.p_data, report->data.len, report->rssi);

  // Wznawia skanowanie, aby kontynuowac nasluchiwanie
  Bluefruit.Scanner.resume();
}

// Inicjalizuje modul BLE i skaner
void ble_init() {
  Serial.println("Inicjalizacja BLE...");
  if (!Bluefruit.begin()) {
    Serial.println("!!! BLAD: Nie udalo sie zainicjalizowac Bluefruit.");
    while (1)
      ;
  }
  Bluefruit.setTxPower(4);
  Bluefruit.setName("LCD Aryia");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  Bluefruit.Scanner.setRxCallback(scan_callback);
  //Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.setInterval(1650, 1600);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);

  Serial.println("Skaner BLE uruchomiony w trybie aktywnym.");
}

// Funkcja, ktora mozna wywolywac w petli loop() w main_sketch.ino
// Ta funkcja przyjmuje DisplayData* i mapuje do nia uśrednioną temperature i RSSI.
void ble_task(DisplayData* data) {
  unsigned long currentTime = millis();

  if (data) {  // Upewnij sie, ze wskaznik nie jest NULL
    // === GLOWNA LOGIKA STATUSOW BEACONA ===
    if (globalDisplayData == nullptr) {
        globalDisplayData = data;
    }
    // 1. Warunek specjalnego alarmu '!' (bardzo dlugo zamrozony licznik zycia sensora)
    if (lastKnownAliveCounter != -1 && (currentTime - lastAliveCounterChangeTime) > (2 * BEACON_ALIVE_COUNTER_TIMEOUT_MS)) {
      Serial.println("BLE_TASK: ALARM! Licznik zycia sensora zamrozony na ponad 2x timeout.");
      data->Alarm.state = '!';       // Ustaw alarm na '!'
      data->Wentylator.state = '!';  // Ustaw status wentylatora rowniez na '!'

      // Dane numeryczne ustawiamy na wartosci "brak danych", ktore zewnetrzny kod moze zinterpretowac
      data->MainTemperature.temperature = -1.0;
      data->MainRssi.RealRssi = -1;


      // Resetujemy akumulatory i ostatnio sparsowane statusy
      accumulatedTempFromBeacons = 0.0;
      accumulatedCountFromBeacons = 0;
      accumulatedRssiFromBeacons = 0.0;
      accumulatedRssiCount = 0;
      // lastParsedAlarmStatus i lastParsedFanStatus NIE sa resetowane tutaj, bo ich wartosc nie ma znaczenia w tym stanie

    }
    // 2. Warunek ogolnego timeoutu (brak DOWOLNYCH danych lub zamrozony licznik na krotki czas)
    else if (currentTime - lastBeaconDataUpdateTime > BEACON_DATA_TIMEOUT_MS || (lastKnownAliveCounter != -1 && currentTime - lastAliveCounterChangeTime > BEACON_ALIVE_COUNTER_TIMEOUT_MS)) {

      Serial.println("BLE_TASK: Dane z beacona niewiarygodne (timeout ogolny lub licznik stoi).");

      // Ustaw dane numeryczne na wartosci "brak danych"
      data->MainTemperature.temperature = -1.0;
      data->MainRssi.RealRssi = -1;


      // Ustaw status wentylatora i alarmu na "!" dla niewiarygodnych danych
      data->Alarm.state = '!';
      data->Wentylator.state = '!';


      // Resetujemy akumulatory
      accumulatedTempFromBeacons = 0.0;
      accumulatedCountFromBeacons = 0;
      accumulatedRssiFromBeacons = 0.0;
      accumulatedRssiCount = 0;
      // lastParsedAlarmStatus i lastParsedFanStatus NIE sa resetowane tutaj

    }
    // 3. Dane sa wiarygodne
    else {
      // Oblicz srednia temperature z akumulatora i zresetuj
      if (accumulatedCountFromBeacons > 0) {
        data->MainTemperature.temperature = accumulatedTempFromBeacons / accumulatedCountFromBeacons;
      } else {
        data->MainTemperature.temperature = -1.0;  // Brak danych temp. w biezacym oknie
      }
      // Zawsze resetuj akumulator temp po odswiezeniu
      accumulatedTempFromBeacons = 0.0;
      accumulatedCountFromBeacons = 0;

      // Oblicz srednia RSSI z akumulatora i zresetuj
      if (accumulatedRssiCount > 0) {
        data->MainRssi.RealRssi = (int)roundf(accumulatedRssiFromBeacons / accumulatedRssiCount);
      } else {
        data->MainRssi.RealRssi = -1;  // Brak sygnalu RSSI w biezacym oknie
      }
      // Zawsze resetuj akumulator RSSI po odswiezeniu
      accumulatedRssiFromBeacons = 0.0;
      accumulatedRssiCount = 0;

      // === Ustaw status wentylatora i alarmu na podstawie sparsowanych danych ===
      // Te statusy sa przekazywane jako char, BEZ MODYFIKACJI source
      data->Wentylator.state = lastParsedBeaconData.fan_status;
      data->Alarm.state = lastParsedBeaconData.alarm_status;
      data->MainTempeSource.AnalogDigital=source;

       static int counterAnalog=0;
        if(lastParsedBeaconData.temp_analog==-1)
        {
            if(counterAnalog++>20)
            {
              counterAnalog=100; 
              data->MainTempeSource.AnalogStatus=-1; 
            }
        }
        else{
            counterAnalog=0;
            data->MainTempeSource.AnalogStatus=1; 
        }
        static int counterDigital=0;
        if(lastParsedBeaconData.temp_digital==-1)
        {
            if(counterDigital++>20)
            {
              counterDigital=100;
              data->MainTempeSource.DigitalStatus=-1; 
            }
            else {
              counterDigital=0;
              globalDisplayData->MainTempeSource.DigitalStatus=1; 
            }
        }


      // =========================================================================
    }
  }
}