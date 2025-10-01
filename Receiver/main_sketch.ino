#include <bluefruit.h>  // ✨ Kluczowa zmiana: dołącz to najpierw
#include "simulation.h"
#include "LCD_Display.h"
#include "bmp280_sensor.h"
#include "imu_sensor.h"
#include "light_sensor_control.h"  // Dodałem ten wiersz, aby naprawić błąd
#include "ble_control.h"           // <-- WAZNE: Dołączenie nowego pliku nagłówkowego
#include "alarm_handler.h"
#include "Config.h"
#include "virtual_button.h"



// --- Zmienne globalne do sterowania kalibracja ---
unsigned long lastCalibrationToggleTime = 0;

// Instancja struktury danych, ktora bedzie wypelniana
DisplayData currentDisplayData;
bool Button = false;

void setup() {
  // Ustawiamy zrodlo danych w setup() - dla IMU zawsze Real

#if 1
  currentDisplayData.MainTemperature.source = Real;
  currentDisplayData.MainAngle.source = Real;
  currentDisplayData.MainRssi.source = Real;
  currentDisplayData.MainHeight.source = Real;
  currentDisplayData.MainVoltage.source = Real;
  currentDisplayData.MainCalibrate.Source = Real;
  currentDisplayData.MainTempeSource.source = Real;
  currentDisplayData.Alarm.source = Real;
  currentDisplayData.Wentylator.source = Real;
  #else
  currentDisplayData.MainTemperature.source = Simul;
  currentDisplayData.MainAngle.source = Simul;
  currentDisplayData.MainRssi.source = Simul;
  currentDisplayData.MainHeight.source = Simul;
  currentDisplayData.MainVoltage.source = Simul;
  currentDisplayData.MainCalibrate.Source = Simul;
  currentDisplayData.MainTempeSource.source = Simul;
  currentDisplayData.Alarm.source = Simul;
  currentDisplayData.Wentylator.source = Simul;
  #endif 


  // Inicjalizujemy na wypadek niepowodzenia odczytu z czujnika
  currentDisplayData.MainAngle.RealAngle = -1;
  currentDisplayData.MainHeight.RealHeght = -1;

  currentDisplayData.MainTempeSource.DigitalStatus=-1;
  currentDisplayData.MainTempeSource.AnalogStatus=-1;


  currentDisplayData.MainCalibrate.CalibrationConuter = 20;

  Serial.begin(115200);
  int counterWait=0;
    
  while (!Serial && counterWait<10)
  {
    delay(100); 
    counterWait++;
  }

  Serial.println("-----------------------------------Starting application--------------------------------------");

  Serial.println("Inicjalizacja LCD...");
  LCD_init();
  Serial.println("LCD zainicjalizowany.");

  setBacklightPWM(255);
  Serial.println("Podswietlenie LCD ustawione na max.");

  if (!initBmp280()) {
    Serial.println(F("Inicjalizacja BMP280 nie powiodla sie! Program bedzie uzywal danych symulacyjnych."));
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, (tft.height() / 2) - 20);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(F("BMP280 init failed!"));
    delay(2000);
    tft.fillScreen(ST77XX_BLACK);
  } else {
    Serial.println(F("BMP280 zainicjowany pomyslnie."));
  }

  // Proba inicjalizacji IMU. Nie zmieniamy zrodla danych, jesli sie nie powiedzie.
  if (!imu_init()) {
    Serial.println(F("Inicjalizacja IMU nie powiodla sie!"));
  } else {
    Serial.println(F("IMU zainicjalizowany pomyslnie."));
  }
  if(LoadImuCalibration()==false)
  {
    int i;
    for(i=0;i<4;i++)
    {
      int timeTone=200;
      tone(BUZZER_PIN, 750, timeTone);
      delay(timeTone*2);
    }
    setCallIMUasZero();  
  }
  // --- WYWOŁANIE FUNKCJI INICJALIZUJĄCEJ BLUETOOTH ---
  ble_init();
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  analogReadResolution(12);
  lastCalibrationToggleTime = millis();
}

void loop() {
  static unsigned long lastUpdateTimestamp = 0;  // Nowa zmienna dla timera
  unsigned long currentTime = millis();

  // 1. Cykliczne zadania - wykonywane co 1000ms
  if (currentTime - lastUpdateTimestamp >= 1000) {
    lastUpdateTimestamp = currentTime;  // Zapisz czas nowej aktualizacji

    // Przetwarzanie danych z BMP280
    Bmp280Data bmpData;
    bool bmpReadSuccess = readBmp280Data(&bmpData);
    if (bmpReadSuccess) {
      currentDisplayData.MainHeight.RealHeght = (int)bmpData.altitude;
    } else {
      currentDisplayData.MainHeight.RealHeght = -1;
    }

    // Przetwarzanie danych z IMU
    if (currentDisplayData.MainAngle.source == Real) {
      IMUData imuData;
      if (readIMUData(&imuData)) {
        currentDisplayData.MainAngle.RealAngle = (int)roundf(/*abs*/(imuData.tilt) / 90.0 * 100.0);
        currentDisplayData.x_acel=imuData.X;
        currentDisplayData.y_acel=imuData.Y;
        currentDisplayData.z_acel=imuData.Z;
        currentDisplayData.MainAngle.Valid=true;
      }
    }
    if (currentDisplayData.MainCalibrate.CalibrationConuter > 0) {
      currentDisplayData.MainCalibrate.CalibrationConuter--;
      if (Button == true || currentDisplayData.MainCalibrate.RealCalibrate == true) {
        setAbsoluteHighest(270);  // Kalibracja wysokosci na 270m
        Call_IMU_Calibration(1);   // Kalibracja zyroskopu z zapisem do QSPI
        Serial.print("kalibaracja czujnikow]\n");
        tone(BUZZER_PIN, 500, 100);
        currentDisplayData.MainCalibrate.RealCalibrate = true;
        if (Button == true) {
          sprintf(currentDisplayData.MainCalibrate.TextValue, "Kalibr Manualna");
        }
        currentDisplayData.MainCalibrate.CalibrationConuter=0;
      } else tone(BUZZER_PIN, 500, 10);
    }

    // Przetwarzanie i symulacja danych
    ble_task(&currentDisplayData);
    Button = readVirtualButton();
    if (Button == true) {
      static int countButton = 0;
      Serial.printf("Push Button %d\n", ++countButton);
      tone(BUZZER_PIN, 1000, 10);
    }
    simulateSensorData(&currentDisplayData, currentTime);
    debugPrintSimulatedData(&currentDisplayData);

    // Obsługa kontroli jasności
    handleBrightnessControl(&currentDisplayData, currentTime);

    static int intTimeerPopupWindow=0;
    intTimeerPopupWindow++;
    if((intTimeerPopupWindow%(60*5))==0)
    {
      //currentDisplayData.MainTempeSource.AnalogStatus=-1;
      //currentDisplayData.MainTempeSource.DigitalStatus=-1;
        currentDisplayData.textPopup[0]=0;
        if(currentDisplayData.MainTempeSource.AnalogStatus==-1)
        {
            sprintf(currentDisplayData.textPopup,"A-> ");
            currentDisplayData.popUpWindow=true;
        }
        if(currentDisplayData.MainTempeSource.DigitalStatus==-1)
        {
            sprintf(currentDisplayData.textPopup+strlen(currentDisplayData.textPopup),"D-> ");
            currentDisplayData.popUpWindow=true;
            
        }
        if(currentDisplayData.popUpWindow==true)
        {
            sprintf(currentDisplayData.textPopup+strlen(currentDisplayData.textPopup),"ERR"); 
            tone(BUZZER_PIN, 1000, 10);       //synalizacja że jest jakiś komunikat
        }
    }
    // Aktualizacja wyświetlacza
    LCD_PUT(&currentDisplayData);
    currentDisplayData.popUpWindow=false;
    currentDisplayData.MainCalibrate.TextValue[0] = 0;
    if (currentDisplayData.MainCalibrate.RealCalibrate == true) {
      currentDisplayData.MainCalibrate.RealCalibrate = false;
    }
  }

  // 2. Obsługa alarmu - WYKONYWANA W KAŻDEJ PĘTLI
  handleAlarm(&currentDisplayData, currentTime, Button);
  CollectImu();
}