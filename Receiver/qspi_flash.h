#ifndef QSPI_FLASH_H
#define QSPI_FLASH_H

#include <Arduino.h>
#include <nrfx_qspi.h>

bool QSPI_mem_connect();
void QSPI_mem_disconnect();
bool QSPI_mem_erase_4KB(uint32_t QSPI_addr);
bool QSPI_mem_write(uint32_t QSPI_addr, void const *p_buffer, size_t byte_size);
bool QSPI_mem_read(uint32_t QSPI_addr, void *p_buffer, size_t byte_size);

#endif