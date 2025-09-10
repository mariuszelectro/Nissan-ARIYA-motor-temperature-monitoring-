#include <bluefruit.h>
#include "analog.h"
#include <cstdio>
#include <string.h>

// Definicje pinów LED
const int LED_RED_PIN = LED_RED;
const int LED_BLUE_PIN = LED_BLUE;

const uint16_t MANUFACTURER_ID = 0x0822; // Identyfikator Adafruit
#define SecYear 86400

//w tym trybie to pobiera 14-15uA w trybie połącznia 500uA
// Deklaracje usług i charakterystyk
BLEUart uart;

// Globalne zmienne, które zachowują wartość po resecie, o ile nie jest to reset z pinu
float inputVoltage = NAN;
int count __attribute__((section(".noinit")));

void updateAdvertisement()
{
  const char* prefix = "BAT:";
  const size_t prefix_len = strlen(prefix);
  
  // Napięcie w mV
  uint16_t voltage_mV = (uint16_t)(inputVoltage * 1000);

  // Przygotuj bufor danych producenta (manufacturer data)
  // Długość: 2 (ID producenta) + prefix_len + 2 (napięcie) + 4 (licznik)
  uint8_t adv_payload[2 + prefix_len + 2 + 4];

  // Kopiowanie ID producenta (0x0822) do bufora w kolejności little-endian
  memcpy(adv_payload, &MANUFACTURER_ID, 2);
  // Kopiowanie "BAT:"
  memcpy(adv_payload + 2, prefix, prefix_len);
  // Kopiowanie napięcia
  memcpy(adv_payload + 2 + prefix_len, &voltage_mV, 2);
  // Kopiowanie licznika
  memcpy(adv_payload + 2 + prefix_len + 2, &count, 4);

  // Wyczyść poprzednie dane reklamowe
  Bluefruit.Advertising.clearData();
  // Dodaj nowy pakiet danych reklamowych (Manufacturer Specific Data)
  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, adv_payload, sizeof(adv_payload));
}

void setup() {
  // SPRAWDŹ REJESTR RESETU ZANIM GO WYCZYŚCISZ
  uint32_t resetReason = NRF_POWER->RESETREAS;
  // Wyczyść rejestr RESETREAS, aby zapobiec fałszywym odczytom w przyszłości
  NRF_POWER->RESETREAS = 0xFFFFFFFF;

  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- Starting Calibrator Bluefruit ---");
  
  if (resetReason & POWER_RESETREAS_RESETPIN_Msk) {
        // Reset z zasilania (pin) - zeruj zmienną
        Serial.println("INFO: Reset z zasilania. Zerowanie zmiennych.");
        count = 0;
  }
  
  // Inicjalizacja diod LED
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  pinMode(LED_BLUE_PIN, OUTPUT);
  // Zgaszenie diody niebieskiej zaraz po starcie
  digitalWrite(LED_BLUE_PIN, HIGH);
  // Dioda na XIAO nRF52840 jest 'active low'
  
  // Konfiguracja pinów do pomiaru napięcia
  setChargeCurrent_100mA();
  enableVoltageDivider();
  // Odczyt napięcia PRZED inicjalizacją BT
  inputVoltage = getBatteryVoltage();


  // Inicjalizacja Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  // Dodanie serwisu BLE UART
  uart.begin();
  
  // Ustawienie nazwy reklamowej
  char nameBuffer[64];
  sprintf(nameBuffer, "BAT:%.3fV,%4dD,%2ds\n", inputVoltage, count/SecYear,count%60);
  Bluefruit.setName(nameBuffer);
  
  // Ustawienie danych reklamowych (teraz z prefiksem)
  updateAdvertisement();
  
  // Rozpoczęcie pierwszego cyklu
  Serial.println("INFO: Uruchomiono tryb pracy cyklicznej.");
  Bluefruit.Advertising.start(0);
}

//to traw ok 2ms
void updateParams(void)
{
    count++;
    inputVoltage = getBatteryVoltage();
}

void loop() {
  // --- Rozpoczęcie cyklu ---
  
  // Jeśli jest połączenie, wysyłaj dane przez UART
  if (Bluefruit.connected()) {
    digitalWrite(LED_RED_PIN, LOW);
    updateParams();
    // Zapal niebieską diodę, aby wskazać połączenie
    digitalWrite(LED_BLUE_PIN, LOW);
    char dataBuffer[64];
    sprintf(dataBuffer, "BAT:%.3fV,%4dD,%2ds\n", inputVoltage, count/86400,count%60 );
    uart.write(dataBuffer);
    Serial.println("INFO: Wyslano dane przez UART.");
    // Kontynuuj miganie diodą
    delay(8);
    digitalWrite(LED_RED_PIN, HIGH);

  } else {
    // Jeśli nie ma połączenia, wyłącz niebieską diodę i reklamuj się
    // Odczyt danych i aktualizacja nazwy reklamowej
    digitalWrite(LED_RED_PIN, LOW);
    char dataBuffer[64];
    sprintf(dataBuffer, "BAT:%.3fV,%4dD,%2ds\n", inputVoltage, count/SecYear,count%60 );
    Bluefruit.setName(dataBuffer); // <-- Dodano prefix do nazwy
    Serial.print(dataBuffer);
    // Upewnij się, że reklamowanie jest aktywne
    Bluefruit.Advertising.start(0);
    // Miganie diodą w momencie startu reklamy
    delay(8);
    updateParams();
    updateAdvertisement(); // <-- zaktualizuj dane reklamowe po zmianie parametrów
    Bluefruit.Advertising.stop();
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_BLUE_PIN, HIGH);
  }
  if ((count%(SecYear*7))==0)     //reset co 7 dni 
  {
    NVIC_SystemReset();
  }
  // Wróć do stanu uśpienia na resztę cyklu, aby oszczędzać energię
  delay(991);
}