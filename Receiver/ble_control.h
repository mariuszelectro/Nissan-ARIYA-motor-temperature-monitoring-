#ifndef BLE_CONTROL_H
#define BLE_CONTROL_H

#include "LCD_Display.h" 
#include <bluefruit.h>

#define MAX_TRACKED_ADDRESSES 20 

#define BEACON_DATA_TIMEOUT_MS          3000UL // Czas (ms) po ktorym dane z beacona uznaje sie za nieaktualne
#define BEACON_ALIVE_COUNTER_TIMEOUT_MS 2000UL // Czas (ms) po ktorym licznik zycia sensora musi sie zmienic

class BLEAddress {
private:
    uint8_t _address[6];
public:
    void set(const uint8_t* address) {
        memcpy(_address, address, 6);
    }
    bool operator==(const ble_gap_addr_t& other) const {
        return memcmp(_address, other.addr, 6) == 0;
    }
    // ✨ Dodano publiczną metodę, która zwraca wskaźnik do prywatnej tablicy adresu
    const uint8_t* getAddress() const {
        return _address;
    }
};

void ble_init();
void ble_task(DisplayData* data);

extern BLEAddress foundAddresses[MAX_TRACKED_ADDRESSES];
extern volatile uint8_t foundAddressCount;
extern volatile int8_t foundRssi[MAX_TRACKED_ADDRESSES]; 
extern volatile bool hasNewBeaconData;
extern DisplayData* globalDisplayData;

#endif // BLE_CONTROL_H