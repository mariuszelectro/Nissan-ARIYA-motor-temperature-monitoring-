#include "virtual_button.h"
#include "Config.h"

// === Definicje stanów i zmienne statyczne automatu stanów ===
enum VirtualButtonState {
    IDLE,              // Stan początkowy, napięcie > 0.2V
    LOW_DETECTED,      // Napięcie spadło poniżej 0.2V, czekamy na kolejny odczyt
    BUTTON_TRIGGERED,  // Przycisk zostal "wciśniety"
};

// Zmienne statyczne, które zachowują wartość między wywołaniami funkcji
static VirtualButtonState currentState = IDLE;
static bool lastVoltageWasLow = false; // Zapamiętujemy poprzedni stan
static int consecutiveLowReads = 0;

// === Definicja progu napięcia ===
const float VIRTUAL_BUTTON_THRESHOLD_V = 0.05;

bool readVirtualButton() {
    // 1. Odczytaj wartość analogową z fotorezystora
    int analogValue = analogRead(LIGHT_SENSOR_PIN);
    
    // 2. Skonwertuj odczyt ADC na napięcie w woltach
    float currentVoltage = analogValue * (ADC_MAX_VOLTAGE / MY_ADC_RESOLUTION_VALUE);

    // Sprawdź, czy aktualne napięcie jest niskie
    bool currentVoltageIsLow = (currentVoltage < VIRTUAL_BUTTON_THRESHOLD_V);

    // Zresetuj flagę wyzwolenia na początku każdego wywołania
    bool isButtonTriggered = false;

    // --- Logika automatu stanów (oparta na 1s cyklu pętli) ---
    switch (currentState) {
        case IDLE:
            // Czekamy na zbocze opadające (przejście z wysokiego na niskie napięcie)
            if (currentVoltageIsLow && !lastVoltageWasLow) {
                consecutiveLowReads = 1; // Rozpoczynamy liczenie
                currentState = LOW_DETECTED;
            }
            break;

        case LOW_DETECTED:
            // Sprawdzamy, czy napięcie jest nadal niskie w kolejnym cyklu
            if (currentVoltageIsLow) {
                consecutiveLowReads++;
                if (consecutiveLowReads >= 2) {
                    // Mamy 2 kolejne odczyty niskiego napięcia (2 sekundy)
                    isButtonTriggered = true;
                    currentState = BUTTON_TRIGGERED; // Przejdź do stanu wyzwolonego
                }
            } else {
                // Napięcie wzrosło, wracamy do IDLE
                consecutiveLowReads = 0;
                currentState = IDLE;
            }
            break;

        case BUTTON_TRIGGERED:
            // Przycisk został wyzwolony, czekamy na jego zwolnienie
            // aby móc ponownie go aktywować
            if (!currentVoltageIsLow) {
                consecutiveLowReads = 0;
                currentState = IDLE;
            }
            break;
    }

    // Zapamiętaj aktualny stan napięcia dla następnej iteracji
    lastVoltageWasLow = currentVoltageIsLow;

    return isButtonTriggered;
}