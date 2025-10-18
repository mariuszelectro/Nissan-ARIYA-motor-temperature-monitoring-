#include "virtual_button.h"
#include "Config.h"
#include <Arduino.h>

// === Definicje stanów i zmienne statyczne automatu stanów ===
enum VirtualButtonState {
    IDLE,               // Stan poczatkowy, oczekiwanie na zbocze opadajace
    LOW_DETECTED,       // Wykryto niskie napiecie, czekamy na potwierdzenie
    BUTTON_PRESSED,     // STAN TRZYMANIA/UCISNIECIA, ZARZADZA JEDNORAZOWA SYGNALIZACJA
};

// Zmienne statyczne, ktore zachowuja wartosc miedzy wywolaniami funkcji
static VirtualButtonState currentState = IDLE;
static int consecutiveLowReads = 0;
static int consecutiveHighReads = 0; 

// --- Zmienne do dynamicznej regulacji progow ---
static float currentBaselineVoltage = 0.0;
// Stałe z pierwotnego pliku/konfiguracji
const float MIN_HIGH_VOLTAGE = 0.3; // Minimalny poziom wysoki 0.3V 
const float LOW_THRESHOLD_MULTIPLIER = 0.4; 
const int CONFIRMATION_READS_COUNT = 3; // Ilosc kolejnych odczytow dla potwierdzenia niskiego stanu
const int CONFIRMATION_HIGH_COUNT = 3; // Liczba kolejnych odczytów dla potwierdzenia stałego powrotu światła (np. 3 sekundy)

// --- ZMIANA: Zmienne dla mechanizmu autozwalniania ---
static int releaseCounter = 0;           // Licznik wywołań (co 1s), zastępuje millis()
const int AUTO_RELEASE_COUNT = 5;        // 5 kroków = 5 sekund
static bool isBiasLocked = false;        // Flaga blokujaca uczenie biasu po auto-zwolnieniu
static bool isFirstCycleInPressed = false; // Flaga: Sygnalizuje pierwsze wciśnięcie

// --- Zmienne do usredniania biasu ---
#define BIAS_SAMPLE_COUNT 10 // Ilosc probek do usrednienia
static float biasSamples[BIAS_SAMPLE_COUNT];
static int biasSampleIndex = 0;
static bool isFirstRead = true; 

// Funkcja pomocnicza do usredniania danych
float calculateAverage(float* array, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += array[i];
    }
    return sum / size;
}

/**
 * @brief Odczytuje stan wirtualnego przycisku opartego na fotorezystorze.
 * @return true, jeśli wirtualny przycisk jest "wcisniety" (tylko w pierwszym cyklu wciśnięcia), w przeciwnym razie false.
 */
bool readVirtualButton() {
    // 1. ODCZYT I KONWERSJA
    int rawValue = analogRead(LIGHT_SENSOR_PIN);
    float currentVoltage = (float)rawValue * (ADC_MAX_VOLTAGE / MY_ADC_RESOLUTION_VALUE);

    // 2. LOGIKA DYNAMICZNEGO BIASU
    if (isFirstRead) {
        
        // Ustalenie progu dynamicznego na podstawie minimalnej wartości, aby umożliwić wykrycie wciśnięcia
        // Jeśli start w ciemności, bias = MIN_HIGH_VOLTAGE, aby stworzyć próg.
        currentBaselineVoltage = (currentVoltage < MIN_HIGH_VOLTAGE) ? MIN_HIGH_VOLTAGE : currentVoltage;
        
        // Uzupełniamy bufor skorygowaną wartością
        for (int i = 0; i < BIAS_SAMPLE_COUNT; i++) {
            biasSamples[i] = currentBaselineVoltage;
        }
        
        // Obliczenie progu do sprawdzenia stanu
        float dynamicLowThreshold_initial = currentBaselineVoltage * LOW_THRESHOLD_MULTIPLIER;
        
        if (currentVoltage <= dynamicLowThreshold_initial) {
            // 🟢 KLUCZOWA ZMIANA: Start w ciemności -> natychmiastowa blokada, czekanie na UNLOCK
            isBiasLocked = true; 
            currentState = IDLE; 
        }
        
        isFirstRead = false;
    } 

    // --- Logika biasu (aktywna tylko w stanie IDLE) ---
    if (currentState == IDLE) {
        
        // Standardowe uczenie biasu działa tylko, gdy NIE jest zablokowane 
        // ORAZ GÓRA, gdy currentVoltage jest powyżej progu jasności (MIN_HIGH_VOLTAGE).
        if (!isBiasLocked && currentVoltage > MIN_HIGH_VOLTAGE) { 
            
            // Natychmiastowa korekcja biasu, jesli napiecie wzroslo
            if (currentVoltage > currentBaselineVoltage) {
                currentBaselineVoltage = currentVoltage;
            }

            // Zbieramy próbki do obliczenia średniej
            biasSamples[biasSampleIndex] = currentVoltage;
            biasSampleIndex = (biasSampleIndex + 1) % BIAS_SAMPLE_COUNT;

            // Obliczenie nowej wartosci biasu po zebraniu pelnej puli probek
            if (biasSampleIndex == 0) {
                currentBaselineVoltage = calculateAverage(biasSamples, BIAS_SAMPLE_COUNT);
            }
        }
        // Jeśli jest ciemno (currentVoltage <= MIN_HIGH_VOLTAGE), bias pozostaje zamrożony na ostatniej znanej wartości.
    }
    
    // Zapewniamy, ze poziom bazowy nigdy nie jest nizszy niz minimalny prog
    if (currentBaselineVoltage < MIN_HIGH_VOLTAGE) {
        currentBaselineVoltage = MIN_HIGH_VOLTAGE;
    }


    // 3. OBLICZENIE PROGU
    float dynamicLowThreshold = currentBaselineVoltage * LOW_THRESHOLD_MULTIPLIER;
    bool currentVoltageIsLow = (currentVoltage <= dynamicLowThreshold);
    bool isButtonTriggered = false; // Ustawiane na false na początku każdego cyklu

    // --- Logika automatu stanow ---
    switch (currentState) {
        case IDLE:
            
            // --- Logika wykrywania stałego powrotu światła (Odblokowanie) ---
            if (isBiasLocked && currentVoltage > MIN_HIGH_VOLTAGE) {
                consecutiveHighReads++;
                if (consecutiveHighReads >= CONFIRMATION_HIGH_COUNT) {
                    isBiasLocked = false;
                    consecutiveHighReads = 0;
                    
                    // Reset biasu do nowej, wysokiej wartości i uzupełnienie bufora
                    for (int i = 0; i < BIAS_SAMPLE_COUNT; i++) {
                        biasSamples[i] = currentVoltage;
                    }
                    biasSampleIndex = 0;
                    currentBaselineVoltage = currentVoltage; // Ustaw nowy bias natychmiast
                }
            } 
            else if (isBiasLocked) {
                // Jeśli jest zablokowany i napięcie nie osiągnęło progu MIN_HIGH
                consecutiveHighReads = 0; // Reset, jeśli błysk minął
            }
            
            // Jeśli bias jest zablokowany, nie pozwalamy na przejście do LOW_DETECTED.
            if (isBiasLocked) {
                break; 
            }
            
            // --- Kontynuacja: Wykrycie wciśnięcia (tylko jeśli nie jest zablokowany) ---
            if (currentVoltageIsLow) {
                consecutiveLowReads = 1;
                currentState = LOW_DETECTED;
            }
            break;

        case LOW_DETECTED:
            consecutiveHighReads = 0; 
            
            if (currentVoltageIsLow) {
                consecutiveLowReads++;
                if (consecutiveLowReads >= CONFIRMATION_READS_COUNT) {
                    isFirstCycleInPressed = true; 
                    currentState = BUTTON_PRESSED; 
                    releaseCounter = 0; 
                }
            } else {
                consecutiveLowReads = 0;
                currentState = IDLE;
            }
            break;
            
        case BUTTON_PRESSED: // STAN TRZYMANIA/UCISNIECIA
            
            consecutiveHighReads = 0; 
            
            // 1. ZARZĄDZANIE JEDNORAZOWYM ZGŁOSZENIEM (true tylko w pierwszym cyklu)
            if (isFirstCycleInPressed) {
                isButtonTriggered = true; 
                isFirstCycleInPressed = false; 
            } 

            releaseCounter++; // Zwiększenie licznika (co 1s)

            // 2. Logika Auto-Zwalniania po 5 sekundach
            if (releaseCounter >= AUTO_RELEASE_COUNT) {
                
                // ZWOLNIENIE: Ustawienie obecnego niskiego napięcia jako NOWY BIAS
                currentBaselineVoltage = currentVoltage; 
                
                // WYPEŁNIJ CAŁY BUFOR NOWYM, NISKIM NAPIĘCIEM
                for (int i = 0; i < BIAS_SAMPLE_COUNT; i++) {
                    biasSamples[i] = currentVoltage; 
                }
                biasSampleIndex = 0; 

                // BLOKOWANIE: ZABLOKUJ UCZENIE BIASU I AKTYWACJĘ DO POWROTU SWIATŁA
                isBiasLocked = true; 
                
                // Reset automatu stanów
                consecutiveLowReads = 0;
                currentState = IDLE;

            } else if (!currentVoltageIsLow) {
                // Alternatywnie: Jeśli przycisk został zwolniony (napięcie wzrosło) przed 5s
                consecutiveLowReads = 0;
                currentState = IDLE;
            }
            break;
    }

    // === WYŚWIETLANIE NA KONSOLI W JEDNEJ LINII PRZY UŻYCIU sprintf ===
    char buffer[130]; 
    sprintf(buffer, "Button: %s | Bias: %.4fV | Threshold: %.4fV | Current: %.4fV | State: %s | HighReads: %d", 
            isButtonTriggered ? "TRUE" : "FALSE",
            currentBaselineVoltage,
            dynamicLowThreshold,
            currentVoltage,
            currentState == IDLE ? (isBiasLocked ? "IDLE_LOCKED" : "IDLE_UNLOCKED") : (currentState == LOW_DETECTED ? "LOW_DETECTED" : "PRESSED/HELD"),
            consecutiveHighReads);
    Serial.println(buffer);

    return isButtonTriggered;
}