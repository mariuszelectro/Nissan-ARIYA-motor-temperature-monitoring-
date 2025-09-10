#include "Print.h"
#include "alarm_handler.h"
#include <Arduino.h>
#include "Config.h"

// Zmienne statyczne do zarządzania stanem alarmu
static bool isAlarmActive = false;
static float initialAlarmTemp = -1.0;
static unsigned long lastValidBeepDuration = 500;
static bool isBuzzerSilenced = false; 

// Zmienne do obsługi automatu stanów buzzera
static bool isBuzzerOn = false;
static unsigned long lastBeepToggleTime = 0;
static unsigned int currentBuzzerFreq = 1000;

void ResetAlarmState(void)
{
        isAlarmActive = false;
        initialAlarmTemp = -1.0;
        lastValidBeepDuration = 500;
        isBuzzerSilenced = false; 
        isBuzzerOn = false;
        lastBeepToggleTime = 0;
        currentBuzzerFreq = 1000;
        noTone(BUZZER_PIN);   
}

void handleAlarm(DisplayData* displayData, unsigned long currentTime, bool isButtonPushed) {
    float currentTemp = displayData->MainTemperature.temperature;
    
    // Warunek aktywacji alarmu (flaga 'A' i poprawne dane). Działa tylko raz.
    bool shouldBeActive = (displayData->Alarm.state == 'A' && currentTemp != -1.0);
    
    // --- LOGIKA AKTYWACJI ALARMU ---
    // Aktywuj alarm tylko, jeśli warunki są spełnione i alarm nie jest jeszcze aktywny.
    if (shouldBeActive && !isAlarmActive) {
        isAlarmActive = true;
        initialAlarmTemp = currentTemp;
        isBuzzerSilenced = false; // Resetujemy stan wyciszenia przy NOWYM alarmie
        currentBuzzerFreq = 1000;
        lastBeepToggleTime = currentTime;
        Serial.printf("Alarm aktywowany! Zapamietana temperatura: %.2fC.\n", initialAlarmTemp);
    }
    
    // --- LOGIKA DEZAKTYWACJI ALARMU ---
    // Alarm zostaje wyłączony tylko, gdy z beacona przyjdzie sygnał 'N' i był aktywny.
    if (displayData->Alarm.state == 'N' && isAlarmActive) {
        ResetAlarmState(); // ✨ KLUCZOWA ZMIANA: używamy Twojej funkcji do pełnego resetu
        Serial.println("Alarm dezaktywowany. Buzzer OFF.");
    }

    // --- LOGIKA Wyciszania Buzera przyciskiem (tylko, jesli alarm jest aktywny) ---
    if (isAlarmActive && isButtonPushed) {
        isBuzzerSilenced = true;
        #ifndef NoSound
        noTone(BUZZER_PIN);
        #endif
        Serial.println("Alarm zostal wyciszony przez wirtualny przycisk.");
    }

    // --- GŁÓWNA LOGIKA CYKLU BUZERA ---
    // Logika działa tylko, gdy alarm jest aktywny
    if (isAlarmActive) {
        unsigned long beepDuration=0;

        // Sekcja ustalania czasu trwania PISZCZENIA
        if (currentTemp != -1.0) 
        {
            // Mamy wiarygodne dane, więc obliczamy czas piszczenia
            if (currentTemp >= ALARM_MAX_TEMP_FOR_BEEP) 
            {
                beepDuration = 1000;
            } else if (currentTemp > initialAlarmTemp) 
            {
                float tempRange = ALARM_MAX_TEMP_FOR_BEEP - initialAlarmTemp;
                float tempIncrease = currentTemp - initialAlarmTemp;
                
                if (tempRange > 0) 
                {
                    float timePerTempUnit = 500.0 / tempRange;
                    beepDuration = (unsigned long)(500 + tempIncrease * timePerTempUnit);
                } else {
                    beepDuration = 500;
                }
            } else {
                beepDuration = 500;
            }
            lastValidBeepDuration = beepDuration;

            // --- SEKCJA USTALANIA CZĘSTOTLIWOŚCI TONU ---
            float tempInterval = (ALARM_MAX_TEMP_FOR_BEEP - initialAlarmTemp) / 4.0;
            if (currentTemp < initialAlarmTemp + tempInterval) {
                currentBuzzerFreq = 800;
            } else if (currentTemp < initialAlarmTemp + 2 * tempInterval) {
                currentBuzzerFreq = 900;
            } else if (currentTemp < initialAlarmTemp + 3 * tempInterval) {
                currentBuzzerFreq = 1000;
            } else {
                currentBuzzerFreq = 1100;
            }
        } else {
            // Brak danych o temperaturze, używamy ostatniego znanego, poprawnego czasu i czestotliwosci
            beepDuration = lastValidBeepDuration;
        }

        unsigned long silenceDuration = 1000 - beepDuration;
        
        // Automat stanów: zarządzanie stanem buzzera
        if (isBuzzerOn) {
            if (currentTime - lastBeepToggleTime >= beepDuration) {
                #ifndef NoSound
                noTone(BUZZER_PIN);
                #endif
                isBuzzerOn = false;
                lastBeepToggleTime = currentTime;
            }
        } else if (!isBuzzerSilenced) { // Sprawdzamy, czy alarm nie jest wyciszony przed włączeniem buzzera
            if (currentTime - lastBeepToggleTime >= silenceDuration) {
                #ifndef NoSound
                tone(BUZZER_PIN, currentBuzzerFreq);
                #endif
                Serial.printf("Buzzer: ON, Czas trwania: %lu ms, Czestotliwosc: %d Hz.\\n", beepDuration, currentBuzzerFreq);
                isBuzzerOn = true;
                lastBeepToggleTime = currentTime;
            }
        }
    }
    // Dodatkowa logika, by wyłączyć buzzer jeśli został wyciszony
    else if (isBuzzerSilenced && isBuzzerOn) {
        #ifndef NoSound
        noTone(BUZZER_PIN);
        #endif
        isBuzzerOn = false;
    }
}