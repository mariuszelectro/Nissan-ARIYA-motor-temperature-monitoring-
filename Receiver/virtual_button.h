#ifndef VIRTUAL_BUTTON_H
#define VIRTUAL_BUTTON_H

#include <Arduino.h>

/**
 * @brief Odczytuje stan wirtualnego przycisku opartego na fotorezystorze.
 * * Funkcja zwraca 'true', jeśli napięcie z fotorezystora spadło poniżej 0.2V
 * z poziomu powyżej 0.2V i pozostawało w tym stanie przez co najmniej 2 sekundy.
 * Działa na zasadzie automatu stanów, co zapewnia, że nie blokuje pętli glownej.
 *
 * @return true, jeśli wirtualny przycisk jest "wcisniety" przez wymagany czas, w przeciwnym razie false.
 */
bool readVirtualButton();

#endif // VIRTUAL_BUTTON_H