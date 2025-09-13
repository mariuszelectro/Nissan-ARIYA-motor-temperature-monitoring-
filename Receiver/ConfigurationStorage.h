#ifndef CONFIGURATION_STORAGE_H
#define CONFIGURATION_STORAGE_H

#include <Arduino.h>
#include <string.h>

bool saveConfig(int16_t numBlock4K, void *current, size_t bsize);
bool loadConfig(int16_t numBlock4K, void *current, void *factory, size_t bsize);

#endif