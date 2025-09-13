#include "ConfigurationStorage.h"
#include "qspi_flash.h"
#include <Arduino.h>
#include <string.h>

static const uint32_t magicValue = 0xCAFE3501;

bool saveConfig(int16_t numBlock4K, void *current, size_t bsize) {
  uint32_t magicNumber = magicValue;
  
  uint32_t qaddr = numBlock4K * 4096;
  uint32_t daddr = qaddr + sizeof(magicNumber);

  Serial.println("Rozpoczynam zapis konfiguracji do pamieci QSPI...");
  if (QSPI_mem_connect()) {
    if (QSPI_mem_erase_4KB(qaddr)) {
      if (QSPI_mem_write(daddr, current, bsize)) {
        if (QSPI_mem_write(qaddr, &magicNumber, sizeof(magicNumber))) {
          Serial.println("Dane konfiguracji zapisane poprawnie.");
          QSPI_mem_disconnect();
          return true;
        }
      }
    }
    QSPI_mem_disconnect();
  }
  
  Serial.println("BLAD: Nie udalo sie zapisac konfiguracji do QSPI!");
  return false;
}

bool loadConfig(int16_t numBlock4K, void *current, void *factory, size_t bsize) {
  memcpy(current, factory, bsize);  
  if (bsize > 4000) return false;

  uint32_t magicNumber;
  uint32_t qaddr = numBlock4K * 4096;
  uint32_t daddr = qaddr + sizeof(magicNumber);

  Serial.println("Rozpoczynam ladowanie konfiguracji z pamieci QSPI...");
  if (QSPI_mem_connect()) {
    if (QSPI_mem_read(qaddr, &magicNumber, sizeof(magicNumber))) {
      if (magicNumber == magicValue) {
        if (QSPI_mem_read(daddr, current, bsize)) {
          QSPI_mem_disconnect();
          return true;
        }
      } else {
        Serial.println("Brak danych konfiguracyjnych w pamieci. Zapisuje wartosci domyslne.");
        saveConfig(numBlock4K, factory, bsize);
        QSPI_mem_disconnect();
        return false;
      }
    }
    QSPI_mem_disconnect();
  }
  
  Serial.println("BLAD: Nie udalo sie odczytac konfiguracji z QSPI!");
  return false;
}