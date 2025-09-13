#include "virtual_button.h"
#include "Config.h"
#include <Arduino.h>

// === Definicje stanów i zmienne statyczne automatu stanów ===
enum VirtualButtonState {
    IDLE,               // Stan poczatkowy, oczekiwanie na zbocze opadajace
    LOW_DETECTED,       // Wykryto niskie napiecie, czekamy na potwierdzenie
    BUTTON_TRIGGERED,   // Przycisk "wciśnięty"
};

// Zmienne statyczne, ktore zachowuja wartosc miedzy wywolaniami funkcji
static VirtualButtonState currentState = IDLE;
static int consecutiveLowReads = 0;

// --- Zmienne do dynamicznej regulacji progow ---
static float currentBaselineVoltage = 0.0;
const float MIN_HIGH_VOLTAGE = 0.3; // Minimalny poziom wysoki 0.3V
const float LOW_THRESHOLD_MULTIPLIER = 0.2; 
const int CONFIRMATION_READS_COUNT = 3; // Ilosc kolejnych odczytow dla potwierdzenia

// --- Zmienne do usredniania biasu ---
#define BIAS_SAMPLE_COUNT 10 // Ilosc probek do usrednienia
static float biasSamples[BIAS_SAMPLE_COUNT];
static int biasSampleIndex = 0;
static bool isFirstRead = true; 

// Funkcja pomocnicza do usredniania danych
float calculateAverage(float* array, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += array[i];
    }
    return sum / size;
}

// === ZMIENIONA FUNKCJA Z NOWĄ LOGIKĄ ZWALNIANIA PRZYCISKU ===
bool readVirtualButton() {
    // 1. Odczytaj wartosc analogowa z fotorezystora
    int analogValue = analogRead(LIGHT_SENSOR_PIN);
    
    // 2. Skonwertuj odczyt ADC na napiecie w woltach
    float currentVoltage = analogValue * (ADC_MAX_VOLTAGE / MY_ADC_RESOLUTION_VALUE);

    // === ZMIANA: Dodatkowe zabezpieczenie na starcie ===
    // Jesli to pierwszy odczyt i napiecie jest zbyt niskie, czekaj na lepsze warunki
    if (isFirstRead) {
        if (currentVoltage < MIN_HIGH_VOLTAGE) {
            // Wypelnij bufor minimalna wartoscia i nie pozwól na aktywacje
            for (int i = 0; i < BIAS_SAMPLE_COUNT; i++) {
                biasSamples[i] = MIN_HIGH_VOLTAGE;
            }
            currentBaselineVoltage = MIN_HIGH_VOLTAGE;

            char buffer[100];
            sprintf(buffer, "Button: IDLE | Waiting for voltage >= %.4fV. Current: %.4fV", 
                    MIN_HIGH_VOLTAGE,
                    currentVoltage);
            Serial.println(buffer);
            return false;
        } else {
            // W przeciwnym wypadku, kontynuuj normalne ladowanie biasu
            currentBaselineVoltage = currentVoltage;
            isFirstRead = false;
            // Wypełniamy bufor początkową wartością
            for (int i = 0; i < BIAS_SAMPLE_COUNT; i++) {
                biasSamples[i] = currentBaselineVoltage;
            }
        }
    }

    // --- Logika biasu (aktywna tylko w stanie IDLE) ---
    if (currentState == IDLE) {
        // Natychmiastowa korekcja biasu, jesli napiecie wzroslo
        if (currentVoltage > currentBaselineVoltage) {
            currentBaselineVoltage = currentVoltage;
        }
        
        // Zbieramy próbki do obliczenia średniej
        biasSamples[biasSampleIndex] = currentVoltage;
        biasSampleIndex++;

        // Obliczenie nowej wartosci biasu po zebraniu pelnej puli probek
        if (biasSampleIndex >= BIAS_SAMPLE_COUNT) {
            biasSampleIndex = 0;
            currentBaselineVoltage = calculateAverage(biasSamples, BIAS_SAMPLE_COUNT);
        }
    }

    // Zapewniamy, ze poziom bazowy nigdy nie jest nizszy niz minimalny prog
    if (currentBaselineVoltage < MIN_HIGH_VOLTAGE) {
        currentBaselineVoltage = MIN_HIGH_VOLTAGE;
    }

    // --- Obliczanie dynamicznego progu niskiego ---
    float dynamicLowThreshold = currentBaselineVoltage * LOW_THRESHOLD_MULTIPLIER;
    bool currentVoltageIsLow = (currentVoltage <= dynamicLowThreshold);
    bool isButtonTriggered = false;

    // --- Logika automatu stanow ---
    switch (currentState) {
        case IDLE:
            if (currentVoltageIsLow) {
                consecutiveLowReads = 1;
                currentState = LOW_DETECTED;
            }
            break;

        case LOW_DETECTED:
            if (currentVoltageIsLow) {
                consecutiveLowReads++;
                if (consecutiveLowReads >= CONFIRMATION_READS_COUNT) {
                    isButtonTriggered = true;
                    currentState = BUTTON_TRIGGERED;
                }
            } else {
                consecutiveLowReads = 0;
                currentState = IDLE;
            }
            break;

        case BUTTON_TRIGGERED:
            if (!currentVoltageIsLow) {
                consecutiveLowReads = 0;
                currentState = IDLE;
            }
            break;
    }

    // === WYŚWIETLANIE NA KONSOLI W JEDNEJ LINII PRZY UŻYCIU sprintf ===
    char buffer[100];
    sprintf(buffer, "Button: %s | Bias: %.4fV | Threshold: %.4fV | Current: %.4fV", 
            isButtonTriggered ? "PRESSED" : "IDLE",
            currentBaselineVoltage,
            dynamicLowThreshold,
            currentVoltage);
    Serial.println(buffer);
    
    return isButtonTriggered;
}