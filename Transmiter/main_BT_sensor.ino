// --------------------------------------------------
// Brief design description:
// Single DS18B20 Temperature Sensor and Analog Voltage Reading with
// Bluetooth Low Energy (BLE) Nordic UART Service (NUS).
// Optimized for Bluefruit Plotter (Voltage, Temperature) and
// Advanced Temperature Alerting with Projected Trend Detection.
// Dynamic BLE Advertising: Updates every 1s with Temp, ADC, and Alarm status.
// Synchronized communication: UART BLE TX and Advertising are 500ms apart.
// LED Behavior:
// - Red LED: Blinks 100ms ON / 900ms OFF (1s cycle). (Control inverted: HIGH=ON)
// - Blue LED: ON when connected. OFF when disconnected/advertising. (Control LOW=ON)
// --------------------------------------------------
// Version: 1.31.0 (Added '----' for unavailable DS18B20 in advertising name)
// Date   : 17/07/2025
// Author : Fire Fox & Gemini AI
// --------------------------------------------------
// Environment:
// - IDE Arduino v2.x (or compatible)
// - Win 10 Pro
// --------------------------------------------------
// Board model: XIAO nRF52840 sense
// --------------------------------------------------
// Useful references:
// https://www.arduino.cc/reference/en/libraries/arduinoble/
// https://github.com/arduino-libraries/ArduinoBLE/
// Nordic UART Service (NUS) UUIDs:
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// RX Characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write)
// TX Characteristic: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Read | Notify)
// --------------------------------------------------

#include <ArduinoBLE.h>            // ArduinoBLE v1.4.0 (or newer)
#define MAXIMWIRE_EXTERNAL_PULLUP  // Use if you have an external pull-up resistor
#include <MaximWire.h>             // MaximWire v1.0.3

#include "NRF52_MBED_TimerInterrupt.h"  // NRF52_MBED_TimerInterrupt v1.4.1
#include <stdlib.h>                     // Still good to keep for general utility
#include <cstdio>                       // Required for snprintf
#include <cmath>                        // Required for round() or other math functions
#include "AnalogTempSensor.h"


// ==================================================
// GLOBAL VARIABLES - DEFINITIONS
// ==================================================
const int ledRedPin = LED_BUILTIN;  // Red LED (D7 on XIAO nRF52)
const int ledBluePin = LEDB;        // Blue LED (D6 on XIAO nRF52)
const int coolingPin = D5;          // Pin controlling cooling (e.g., a relay)

//#define BLEDebug          //nie wyswietlmy info od BLE

// Variables for red LED blinking
#define RED_LED_BLINK_INTERVAL_MS 100  // Timer interval for red LED

unsigned long previousMillisAnalogRead = 0;
volatile unsigned int redLedPhaseCounter = 0;  // Red LED blink phase counter

// Analog input(s) definition



// Dwie niezależne zmienne dla temperatur
static float analogTemperature = NAN;
float ds18b20Temp = NAN;
float inputVoltage = NAN;

// DS18B20 temperature sensor (for a SINGLE sensor)
MaximWire::Bus bus(2);          // OneWire bus on pin D2
MaximWire::DS18B20 tempSensor;  // Object for a single DS18B20 sensor
bool ds18b20_found = false;     // Flag if sensor was found and initialized

// Timers for LEDs (ITimer0 for red)
NRF52_MBED_Timer ITimer0(NRF_TIMER_3);  // For red LED

// Timer for sensor readings and UART BLE TX
unsigned long previousMillisSensorRead = 0;
const long int TMR_READ_SENSORS_MS = 1000;  // Read sensors and send UART BLE every 1s

// Variable to control blue LED after sensor readings
const long int BLUE_LED_DURATION_MS = 50;  // Blue LED blink duration
unsigned long blueLedToggleMillis = 0;     // Time when blue LED was last turned on for a blink

// BLE: board name and device name definition
const char* boardName = "TempSensor";
const char* devName = "AryiaMariusz";  // Pełna nazwa urządzenia (widoczna po połączeniu)

// BLE: Nordic UART Service (NUS) definition
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse | BLEWrite | BLERead, 20);
BLECharacteristic txCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLERead | BLENotify, 20);

// ==================================================
// VARIABLES FOR TREND DETECTION AND NOTIFICATIONS
// ==================================================
#define TREND_WINDOW_SECONDS 30
#define TEMP_HISTORY_SIZE TREND_WINDOW_SECONDS
#define PROJECTION_TIME_SECONDS 10

// Dwie niezależne historie dla każdego czujnika
float tempHistoryDS[TEMP_HISTORY_SIZE];
unsigned long timeHistoryDS[TEMP_HISTORY_SIZE];
int historyIndexDS = 0;
bool historyFullDS = false;

float tempHistoryAnalog[TEMP_HISTORY_SIZE];
unsigned long timeHistoryAnalog[TEMP_HISTORY_SIZE];
int historyIndexAnalog = 0;
bool historyFullAnalog = false;

// Alarm thresholds
const float TEMP_HIGH_THRESHOLD = 70.0;
const float COOLING_OFF_THRESHOLD_PERCENT = 0.10;
float coolingOffThreshold;

// Flagi statusu alarmu dla każdego źródła
bool ds18b20AlertActive = false;
bool analogAlertActive = false;

char alarmStatusChar = 'N';      // 'A' for alarm, 'N' for no alarm
char d5ControlStatusChar = 'n';  // 'W' for fan ON, 'n' for fan OFF
unsigned char MyCounter = 0;     // Twój licznik, brawo za dodanie!

// ==================================================
// FUNCTIONS SECTION
// ==================================================

// Helper function to send notifications via BLE UART
void sendNotification(String message) {
  if (txCharacteristic.subscribed()) {
    #ifdef BLEDebug
    Serial.print("INFO BLE TX Notification: Sending: '" + message + "' (Length: " + message.length() + " bytes)");
  #endif
    if (message.length() > 20) {
      Serial.println(" - INFO: Notification > 20 bytes. Relying on DLE for full delivery.");
    }
    txCharacteristic.writeValue(message.c_str());
    #ifdef BLEDebug
    Serial.println(" - Sent OK!");
    #endif
  } else {
    #ifdef BLEDebug
    Serial.println("INFO BLE TX: Not subscribed, notification not sent.");
    #endif
  }
}

// TIMER0: interrupt handler definition (for red LED)
void TimerHandler0() {
  redLedPhaseCounter++;
  if (redLedPhaseCounter == 1) {
    digitalWrite(ledRedPin, HIGH);
  } else if (redLedPhaseCounter == 10) {
    digitalWrite(ledRedPin, LOW);
    redLedPhaseCounter = 0;
  }
}

// Function to update advertising data
void updateAdvertisingData() {
  // Bufor na temperaturę analogową w formacie "Axx.x" (zawsze 5 znaków: A + 3 cyfry + kropka + 1 cyfra)
  char anTempStr[7];
  if (isnan(analogTemperature)) {
    strcpy(anTempStr, "A----");
  } else {
    int intPart = (int)analogTemperature;
    int decPart = abs((int)(round((analogTemperature - intPart) * 10)));
    if (intPart < 10) {
      snprintf(anTempStr, sizeof(anTempStr), "A  %d.%1d", intPart, decPart);
    } else if (intPart < 100) {
      snprintf(anTempStr, sizeof(anTempStr), "A %d.%1d", intPart, decPart);
    } else {
      snprintf(anTempStr, sizeof(anTempStr), "A%d.%1d", intPart, decPart);
    }
  }

  // Bufor na temperaturę cyfrową w formacie "Cyy.y" (zawsze 5 znaków)
  char dsTempStr[7];
  // POPRAWKA: Dodajemy sprawdzenie NaN dla DS18B20 w nazwie reklamowej
  if (isnan(ds18b20Temp)) {
    strcpy(dsTempStr, "C----");
  } else {
    int intPart = (int)ds18b20Temp;
    int decPart = abs((int)(round((ds18b20Temp - intPart) * 10)));
    if (intPart < 10) {
      snprintf(dsTempStr, sizeof(dsTempStr), "C  %d.%1d", intPart, decPart);
    } else if (intPart < 100) {
      snprintf(dsTempStr, sizeof(dsTempStr), "C %d.%1d", intPart, decPart);
    } else {
      snprintf(dsTempStr, sizeof(dsTempStr), "C%d.%1d", intPart, decPart);
    }
  }

  // Tworzenie dynamicznej nazwy reklamowej w formacie "Axx.x Cyy.y ASC S Z"
  // Przykład: "A 25.1 C 24.8 NN0" (AS=N, CS=n, MyCounter=0) lub "A---- C---- AN0"
  // Długość: 5 (anTempStr) + 1 (spacja) + 5 (dsTempStr) + 1 (spacja) + 1 (AS) + 1 (CS) + 1 (MyCounter) = 15 znaków + null terminator.
  char advNameBuffer[20];  // Bufor (nadal zostawiamy większy bufor na wszelki wypadek)
  int len = snprintf(advNameBuffer, sizeof(advNameBuffer), "%s %s %c%c%d",
                     anTempStr, dsTempStr, alarmStatusChar, d5ControlStatusChar, ((MyCounter++) % 10));

  Serial.print("DEBUG ADV: Analog Temp Str: '");
  Serial.print(anTempStr);
  Serial.print("' DS18B20 Temp Str: '");
  Serial.print(dsTempStr);
  Serial.print("' Alarm Status: '");
  Serial.print(alarmStatusChar);
  Serial.print("' D5 Control Status: '");
  Serial.print(d5ControlStatusChar);
  Serial.println("'");
  Serial.print("DEBUG ADV: Generated Local Name: '");
  Serial.print(advNameBuffer);
  Serial.print("' (Length: ");
  Serial.print(len);
  Serial.println(" bytes)");

  if (len < sizeof(advNameBuffer) && len >= 0) {
    BLE.setLocalName(advNameBuffer);
    #ifdef BLEDebug
    Serial.println("INFO BLE ADV: Local Name (Advertising Name) set successfully.");
    #endif
  } else {
    #ifdef BLEDebug
    Serial.println("ERROR BLE ADV: Generated Local Name is too long or snprintf failed!");
    #endif
    BLE.setLocalName("ERR");
  }

  // UWAGA: Ta część kodu jest kluczowa. Reklama jest uruchamiana tylko wtedy,
  // gdy urządzenie nie jest połączone.
  if (!BLE.central()) {
    // Sprawdź, czy reklama jest już aktywna. Jeśli tak, zatrzymaj ją przed aktualizacją.
    if (BLE.advertise()) {  // Używamy BLE.advertising() do SPRAWDZANIA statusu
      BLE.stopAdvertise();
      #ifdef BLEDebug
      Serial.println("INFO BLE ADV: Stopped existing advertising to update Local Name.");
      #endif
    }

    BLE.advertise();  // Używamy BLE.advertise() do ROZPOCZĘCIA/WZNOWIENIA reklamy
    #ifdef BLEDebug
    Serial.println("INFO BLE ADV: Started/Resumed advertising with dynamic Local Name and Service UUID.");
    #endif
  }
}

// BLE: connection event handler (callback)
void blePeripheralConnectHandler(BLEDevice central) {
  Serial.print("INFO BLE: Connected to central: ");
  Serial.println(central.address());

  Serial.println("INFO BLE: MTU negotiation handled automatically by ArduinoBLE.");

  BLE.stopAdvertise();
  Serial.println("INFO BLE ADV: Stopped advertising on connect.");
  digitalWrite(ledBluePin, LOW);  // TURN ON blue LED permanently when connected (LOW=ON)
}

// BLE: disconnection event handler (callback)
void blePeripheralDisconnectHandler(BLEDevice central) {
  Serial.print("INFO BLE: Disconnected from central: ");
  Serial.println(central.address());
  Serial.println("INFO BLE ADV: Resuming advertising after disconnect.");

  digitalWrite(ledBluePin, HIGH);  // TURN OFF blue LED permanently when disconnected (HIGH=OFF)
  updateAdvertisingData();
}

// Function to handle writing to RX characteristic (when phone sends data to module)
void rxCharWriteHandler(BLEDevice central, BLECharacteristic characteristic) {
  String receivedData = "";
  if (characteristic.valueLength() > 0) {
    for (int i = 0; i < characteristic.valueLength(); i++) {
      receivedData += (char)characteristic.value()[i];
    }
  }

  Serial.print("INFO BLE RX: Received data: ");
  Serial.println(receivedData);

  if (receivedData.length() > 0) {
    char command = receivedData.charAt(0);
    if (command == '1') {
      Serial.println("INFO: Red LED ON via BLE UART");
      digitalWrite(ledRedPin, HIGH);
    } else if (command == '0') {
      Serial.println("INFO: Red LED OFF via BLE UART");
      digitalWrite(ledRedPin, LOW);
    }
  } else {
    Serial.println("INFO BLE RX: Empty data received.");
  }
}

// Funkcja do analizy trendu dla konkretnego źródła temperatury
float analyzeTemperatureTrend(float currentTemp, float* history, unsigned long* timeHistory,
                              int& historyIndex, bool& historyFull, unsigned long currentMillis) {

  float projectedTemp = currentTemp;

  if (!isnan(currentTemp)) {
    history[historyIndex] = currentTemp;
    timeHistory[historyIndex] = currentMillis;
    historyIndex = (historyIndex + 1) % TEMP_HISTORY_SIZE;
    if (historyIndex == 0) {
      historyFull = true;
    }
  } else {
    historyFull = false;
    historyIndex = 0;
    for (int i = 0; i < TEMP_HISTORY_SIZE; ++i) history[i] = NAN;
  }

  if (historyFull && !isnan(currentTemp)) {
    int oldestIndex = historyIndex;
    float oldestTemp = history[oldestIndex];
    unsigned long oldestTime = timeHistory[oldestIndex];

    if (!isnan(oldestTemp) && currentMillis - oldestTime >= (TREND_WINDOW_SECONDS / 2 * 1000UL)) {
      float timeDiffSeconds = (float)(currentMillis - oldestTime) / 1000.0;
      float tempDiff = currentTemp - oldestTemp;

      if (timeDiffSeconds > 0.1 && tempDiff > 0.0) {
        float tempRateOfChange = tempDiff / timeDiffSeconds;
        projectedTemp = currentTemp + (tempRateOfChange * PROJECTION_TIME_SECONDS);
      }
    }
  }
  return projectedTemp;
}


// ==================================================
// setup() function
// ==================================================
void setup() {
  Serial.begin(115200);
  // //while (!Serial) {
  //   delay(10);
  // }
  Serial.println("\n--- Starting MyFinalTempSensor v1.31.0 (Added '----' for unavailable DS18B20 in advertising name) ---");

  coolingOffThreshold = TEMP_HIGH_THRESHOLD * (1.0 - COOLING_OFF_THRESHOLD_PERCENT);
  Serial.print("INFO Cooling: Cooling will turn OFF below: ");
  Serial.print(coolingOffThreshold, 1);
  Serial.println("C");

  for (int i = 0; i < TEMP_HISTORY_SIZE; ++i) {
    tempHistoryDS[i] = NAN;
    timeHistoryDS[i] = 0;
    tempHistoryAnalog[i] = NAN;
    timeHistoryAnalog[i] = 0;
  }

  // --------------------------------------------------
  // DS18B20 temperature sensor initialization
  // --------------------------------------------------
  Serial.println("INFO DS18B20: Searching for single sensor...");
  MaximWire::Discovery discovery = bus.Discover();
  MaximWire::Address currentAddress;

  if (discovery.FindNextDevice(currentAddress)) {
    Serial.print("INFO DS18B20: Sensor found ");
    Serial.print(currentAddress.ToString());

    if (currentAddress.GetModelCode() == MaximWire::DS18B20::MODEL_CODE) {
      Serial.println(" (DS18B20 detected and initialized)");
      tempSensor = MaximWire::DS18B20(currentAddress);
      ds18b20_found = true;
    } else {
      Serial.println(" (Not a DS18B20 sensor or invalid address)");
      ds18b20_found = false;
    }
  } else {
    Serial.println("WARNING DS18B20: No DS18B20 sensor found!");
    ds18b20_found = false;
  }
  Serial.print("DEBUG DS18B20: ds18b20_found is now: ");
  Serial.println(ds18b20_found ? "true" : "false");

  // --------------------------------------------------
  // LED & Control Pins: set output pin used for led(s) and cooling
  // --------------------------------------------------
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);
  pinMode(coolingPin, OUTPUT);

  digitalWrite(ledRedPin, LOW);
  digitalWrite(ledBluePin, HIGH);
  digitalWrite(coolingPin, LOW);

  // --------------------------------------------------
  // TIMER0: set ISR handler and timer interval in microsecs (for red LED)
  // --------------------------------------------------
  if (ITimer0.attachInterruptInterval(RED_LED_BLINK_INTERVAL_MS * 1000, TimerHandler0)) {
    Serial.println("INFO TIMER0: starting ITimer0 OK! (Red LED blink)");
    ITimer0.restartTimer();
  } else {
    Serial.println("ERROR TIMER0: can't set ITimer0. Select another freq. or timer");
  }
  analogTempSensor_init();


  // --------------------------------------------------
  // BLE: INIT initialization - Przeniesione na początek setup()
  // --------------------------------------------------
  Serial.println("INFO BLE: Starting BLE initialization...");
  if (!BLE.begin()) {
    Serial.println("ERROR BLE: initialization failed!");
    while (1)
      ;
  }
  Serial.println("INFO BLE: BLE initialized successfully.");

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Ustaw Pełną Nazwę Urządzenia (widoczna po połączeniu)
  BLE.setDeviceName(devName);
  Serial.print("INFO BLE: Device Name set to: ");
  Serial.println(devName);

  // Dodaj Service UUID do reklamowania
  BLE.setAdvertisedService(uartService);
  Serial.println("INFO BLE: Advertised Service UUID (NUS) set.");

  // Dodaj charakterystyki do usługi
  uartService.addCharacteristic(rxCharacteristic);
  uartService.addCharacteristic(txCharacteristic);
  BLE.addService(uartService);
  Serial.println("INFO BLE: NUS Characteristics added to Service and BLE.");

  // Ustaw początkowe dane reklamowe (pierwsza nazwa z danymi)
  updateAdvertisingData();

  // Pierwsze advertise() jest już w updateAdvertisingData()
}

// ==================================================
// loop() function
// ==================================================
void loop() {
  BLE.poll();

  BLEDevice central = BLE.central();
  int analogReadValue = 0;  //ty przechowywujem wartosć temperaturzy zmierznonej w ADC
  unsigned long currentMillis = millis();

  if (rxCharacteristic.written()) {
    rxCharWriteHandler(central, rxCharacteristic);
  }

  // Kontrolka niebieskiej diody LED
  if (central) {
    digitalWrite(ledBluePin, LOW);  // Włączony, gdy połączono
  } else {
    // Mignij niebieską diodą LED, gdy odczytujemy sensory i nie jesteśmy połączeni
    if (digitalRead(ledBluePin) == LOW && (currentMillis - blueLedToggleMillis >= BLUE_LED_DURATION_MS)) {
      digitalWrite(ledBluePin, HIGH);  // Wyłącz po krótkim czasie
    }
  }
  if (currentMillis - previousMillisAnalogRead >= TMR_READ_ANALOG_MS) {
    previousMillisAnalogRead = currentMillis;
    collectTempProbe();
  }

  if (currentMillis - previousMillisSensorRead >= TMR_READ_SENSORS_MS) {
    previousMillisSensorRead = currentMillis;

    // Odczyty z sensorów

    int analogTemperatureInt = getFilteredAnalogTemperature(&inputVoltage);
  

    if (analogTemperatureInt != -1) {
      analogTemperature = analogTemperatureInt / 100.0;
    }


    Serial.print("Volt_RAW: ");
    Serial.print(analogReadValue);
    Serial.print(", Scaled Volt: ");
    Serial.print(inputVoltage, 2);
    Serial.print("V, Analog Temp: ");
    Serial.print(analogTemperature, 1);
    Serial.println("C");

    if (ds18b20_found) {
      float tempReadingDS18B20 = tempSensor.GetTemperature<float>(bus);
      tempSensor.Update(bus);

      if (!isnan(tempReadingDS18B20)) {
        ds18b20Temp = constrain(tempReadingDS18B20, 0.0, 140.0);
        Serial.print("Temp (DS18B20): ");
        Serial.print(ds18b20Temp, 1);
        Serial.println("C");
      } else {
        Serial.println("ERROR DS18B20: Could not read temperature data! ds18b20Temp is NaN.");
        ds18b20Temp = NAN;
      }
    } else {
      Serial.println("WARNING DS18B20: No sensor initialized or found for reading.");
      ds18b20Temp = NAN;
    }

    Serial.print("DEBUG SENSORS: DS18B20 Temp=");
    Serial.print(ds18b20Temp, 2);
    Serial.print(", Analog Temp=");
    Serial.print(analogTemperature, 2);
    Serial.print(" Current Volt=");
    Serial.println(inputVoltage, 2);

    // Mignięcie niebieskiej diody LED, jeśli nie połączono i są dane
    if (!central && (!isnan(ds18b20Temp) || !isnan(analogTemperature)) && !isnan(inputVoltage)) {
      digitalWrite(ledBluePin, LOW);
      blueLedToggleMillis = currentMillis;
    }

    // Analiza trendu
    float projectedTempDS18B20 = analyzeTemperatureTrend(ds18b20Temp, tempHistoryDS, timeHistoryDS,
                                                         historyIndexDS, historyFullDS, currentMillis);
    float projectedTempAnalog = analyzeTemperatureTrend(analogTemperature, tempHistoryAnalog, timeHistoryAnalog,
                                                        historyIndexAnalog, historyFullAnalog, currentMillis);

    Serial.print("DEBUG TREND: Projected DS18B20 Temp: ");
    Serial.print(projectedTempDS18B20, 2);
    Serial.print(", Projected Analog Temp: ");
    Serial.println(projectedTempAnalog, 2);

    // Sprawdzenie alarmów
    ds18b20AlertActive = false;
    analogAlertActive = false;

    if (!isnan(ds18b20Temp) && ds18b20Temp >= TEMP_HIGH_THRESHOLD) {
      ds18b20AlertActive = true;
      Serial.println("ALERT: DS18B20 Temp reached threshold!");
    } else if (!isnan(projectedTempDS18B20) && projectedTempDS18B20 >= TEMP_HIGH_THRESHOLD && !isnan(ds18b20Temp) && ds18b20Temp < TEMP_HIGH_THRESHOLD) {
      ds18b20AlertActive = true;
      Serial.println("ALERT: Rising trend for DS18B20 predicts temp will reach threshold soon!");
    }

    if (!isnan(analogTemperature) && analogTemperature >= TEMP_HIGH_THRESHOLD) {
      analogAlertActive = true;
      Serial.println("ALERT: Analog Temp reached threshold!");
    } else if (!isnan(projectedTempAnalog) && projectedTempAnalog >= TEMP_HIGH_THRESHOLD && !isnan(analogTemperature) && analogTemperature < TEMP_HIGH_THRESHOLD) {
      analogAlertActive = true;
      Serial.println("ALERT: Rising trend for Analog Temp predicts temp will reach threshold soon!");
    }

    if (ds18b20AlertActive || analogAlertActive) {
      alarmStatusChar = 'A';
    } else {
      alarmStatusChar = 'N';
    }


    // Sterowanie chłodzeniem
    if (digitalRead(coolingPin) == LOW) {             // Chłodzenie WYŁĄCZONE
      if (ds18b20AlertActive || analogAlertActive) {  // Włącz, jeśli któryś alarm aktywny
        digitalWrite(coolingPin, HIGH);
        d5ControlStatusChar = 'W';
        Serial.println("INFO D5 Control: Cooling ON!");
      }
    } else {  // Chłodzenie WŁĄCZONE
      bool ds18b20BelowHysteresis = (!isnan(ds18b20Temp) && ds18b20Temp <= coolingOffThreshold && projectedTempDS18B20 <= TEMP_HIGH_THRESHOLD);
      if (isnan(ds18b20Temp) && !ds18b20AlertActive) ds18b20BelowHysteresis = true;

      bool analogBelowHysteresis = (!isnan(analogTemperature) && analogTemperature <= coolingOffThreshold && projectedTempAnalog <= TEMP_HIGH_THRESHOLD);
      if (isnan(analogTemperature) && !analogAlertActive) analogBelowHysteresis = true;

      if (!ds18b20AlertActive && !analogAlertActive && ds18b20BelowHysteresis && analogBelowHysteresis) {
        digitalWrite(coolingPin, LOW);
        d5ControlStatusChar = 'n';
        Serial.println("INFO D5 Control: Cooling OFF!");
      }
    }
    float zasilane=PomiarZsailania();
    //float zasilane =14.5;
    // Wysyłanie danych przez BLE UART lub aktualizacja reklamy
    if (central) {  // Jeśli połączono, wysyłaj dane przez UART
      String plotData = String(!isnan(analogTemperature) ? analogTemperature : 0.0, 1) + "," + String(!isnan(ds18b20Temp) ? ds18b20Temp : 0.0, 1) + "," + String(zasilane, 1) + "," + String(TEMP_HIGH_THRESHOLD, 1) + "\n";
      if (txCharacteristic.subscribed()) {
        Serial.print("INFO BLE TX Plotter: Sending: '" + plotData.substring(0, plotData.length() - 1) + "' (Length: " + plotData.length() + " bytes)");
        txCharacteristic.writeValue(plotData.c_str());
        Serial.println(" - Sent OK!");
      } else {
        Serial.println("INFO BLE TX: Not subscribed, data not sent (subsc. required).");
      }
    } else {  // Jeśli nie połączono, aktualizuj dane reklamowe
      //char infoBuffer[64];
      //sprintf(infoBuffer,"co za kupa %f\n",analogTemperature);
      //Serial.println(infoBuffer);
      
      updateAdvertisingData();

      //sprintf(infoBuffer,"co za dupa %f\n",analogTemperature);

    }
  }
}