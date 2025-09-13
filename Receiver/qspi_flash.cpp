#include "qspi_flash.h"
#include <Arduino.h>
#include <nrfx_qspi.h>

// Ustawienia PIN-ów QSPI
static nrfx_qspi_config_t QSPI_mem_config = NRFX_QSPI_DEFAULT_CONFIG(21, 25, 20, 24, 22, 23);
static const uint32_t magicValue = 0xCAFE3501;

// Podlaczenie do pamieci QSPI
bool QSPI_mem_connect() {
  return (nrfx_qspi_init(&QSPI_mem_config, NULL, NULL) == NRFX_SUCCESS);
}

// Rozlaczenie z pamiecia QSPI
void QSPI_mem_disconnect() {
  nrfx_qspi_uninit();
}

// Sprawdzenie, czy pamiec jest zajeta (niepotrzebne, ale przydatne)
// bool QSPI_mem_ready() {
//   return (nrfx_qspi_mem_busy_check() == NRFX_SUCCESS);
// }

// Czekaj az pamiec bedzie gotowa (niepotrzebne, ale przydatne)
// bool QSPI_mem_wait_ready() {
//   while (!QSPI_mem_ready()) {
//     /* Wait */
//   }
//   return (true);
// }

// Funkcja kasowania calej pamieci
bool QSPI_mem_erase_all() {
  return (nrfx_qspi_chip_erase() == NRFX_SUCCESS);
}

// Funkcja kasowania bloku 4KB
bool QSPI_mem_erase_4KB(uint32_t QSPI_addr) {
  return (nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, QSPI_addr) == NRFX_SUCCESS);
}

// Funkcja kasowania bloku 64KB (niepotrzebne w tym przypadku)
// bool QSPI_mem_erase_64KB(uint32_t QSPI_addr) {
//   return (nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, QSPI_addr) == NRFX_SUCCESS);
// }

// Odczyt danych z pamieci
bool QSPI_mem_read(uint32_t QSPI_addr, void *p_buffer, size_t byte_size) {
  return (nrfx_qspi_read(p_buffer, byte_size, QSPI_addr) == NRFX_SUCCESS);
}

// Zapis danych do pamieci
bool QSPI_mem_write(uint32_t QSPI_addr, void const *p_buffer, size_t byte_size) {
  return (nrfx_qspi_write(p_buffer, byte_size, QSPI_addr) == NRFX_SUCCESS);
}

// Zapis konfiguracji z uzyciem "Magic Number"
bool QSPI_mem_writeConfig(int16_t numBlock4K, void *current, size_t bsize) {
  uint32_t magicNumber = magicValue;
  
  uint32_t qaddr = numBlock4K * 4096;
  uint32_t daddr = qaddr + sizeof(magicNumber);

  if (QSPI_mem_erase_4KB(qaddr)) {
    if (QSPI_mem_write(daddr, current, bsize)) {
      return (QSPI_mem_write(qaddr, &magicNumber, sizeof(magicNumber)));
    }
  }

  Serial.println("[ERR] QSPI Memory write config");
  return (false);
}

// Odczyt konfiguracji z uzyciem "Magic Number"
bool QSPI_mem_readConfig(int16_t numBlock4K, void *current, void *factory, size_t bsize) {
  // W przypadku bledu, ladowane sa wartosci domyslne
  memcpy(current, factory, bsize);  
  if (bsize > 4000) return (false);

  uint32_t magicNumber;
  uint32_t qaddr = numBlock4K * 4096;
  uint32_t daddr = qaddr + sizeof(magicNumber);

  if (QSPI_mem_read(qaddr, &magicNumber, sizeof(magicNumber))) {
    if (magicNumber == magicValue) {
      return (QSPI_mem_read(daddr, current, bsize));
    } else {
      return (QSPI_mem_writeConfig(numBlock4K, factory, bsize));
    }
  }

  Serial.println("[ERR] QSPI Memory read config");
  return (false);
}

// Wyrzucono funkcje QSPI_mem_dump