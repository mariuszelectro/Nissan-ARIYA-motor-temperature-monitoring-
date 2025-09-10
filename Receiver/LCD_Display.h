#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h> // Standardowe definicje Arduino
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// Wlaczamy Free Fonts
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h> // Poprawiony naglowek

// --- Definicje Pinow dla XIAO nRF52840 ---
#define TFT_CS      3   // Chip select pin (CS)
#define TFT_DC      6   // Data/Command pin (DC)
#define TFT_RST     7   // Reset pin (RST) - lub -1 jesli polaczony z resetem MCU
#define TFT_BL      D0  // Pin podswietlenia (D0/P0.02)
//#define LIGHT_SENSOR_PIN A1 // Pin dla fotorezystora

// --- Definicje Wymiarow Wyswietlacza ---
#define ST7789_PHYSICAL_WIDTH   240
#define ST7789_PHYSICAL_HEIGHT  280
#define ST7789_PHYSICAL_Y_OFFSET 20

// Efektywna rozdzielczosc po obrocie 90 stopni (logicznie poziomy)
#define SCREEN_WIDTH_ROTATED    280
#define SCREEN_HEIGHT_ROTATED   240

// --- Definicje Obszarow Ekranu (po obrocie logicznym) ---
#define TITLE_AREA_HEIGHT_PX        20
#define TEST_WINDOWS_AREA_HEIGHT_PX 80
#define SEPARATOR_HEIGHT_PX         3

#define ALARM_X_POS             100 // Pozycja X dla ikony Alarmu (przykladowa wartosc)
#define ALARM_WIDTH             20  // Szerokosc obszaru ikony alarmu
#define WENTYLATOR_X_POS        (ALARM_X_POS + ALARM_WIDTH) // Pozycja X dla ikony Wentylatora
#define WENTYLATOR_WIDTH        20  // Szerokosc obszaru ikony wentylatora

#define GRAPH_AREA_HEIGHT_PX        (SCREEN_HEIGHT_ROTATED - TITLE_AREA_HEIGHT_PX - SEPARATOR_HEIGHT_PX - TEST_WINDOWS_AREA_HEIGHT_PX)

#define TITLE_Y_START_ON_SCREEN         0
#define GRAPH_Y_START_ON_SCREEN         TITLE_AREA_HEIGHT_PX
#define SEPARATOR_Y_START_ON_SCREEN     (GRAPH_Y_START_ON_SCREEN + GRAPH_AREA_HEIGHT_PX)
#define TEST_WINDOWS_Y_START_ON_SCREEN  (SEPARATOR_Y_START_ON_SCREEN + SEPARATOR_HEIGHT_PX)

// Wewnetrzne definicje dla ST7789 dla funkcji przewijania sprzetowego
#define TFA_SCROLL_PARAM    (ST7789_PHYSICAL_Y_OFFSET + TITLE_AREA_HEIGHT_PX)
#define BFA_SCROLL_PARAM    (TEST_WINDOWS_AREA_HEIGHT_PX + ST7789_PHYSICAL_Y_OFFSET)

// Dodatkowe definicje komend ST77XX (jesli nie zdefiniowane w bibliotece)
#ifndef ST77XX_VSCRDEF
#define ST77XX_VSCRDEF 0x33
#endif
#ifndef ST77XX_VSCSAD
#define ST77XX_VSCSAD 0x37
#endif

// --- Definicje Okien Pomiarowych w Obszarze BFA ---
#define TEMP_WINDOW_WIDTH_PX    140
#define TEMP_WINDOW_HEIGHT_PX   TEST_WINDOWS_AREA_HEIGHT_PX
#define HEIGHT_WINDOW_WIDTH_PX  80
#define HEIGHT_WINDOW_HEIGHT_PX TEST_WINDOWS_AREA_HEIGHT_PX
#define ANGLE_WINDOW_WIDTH_PX   60
#define ANGLE_WINDOW_HEIGHT_PX  TEST_WINDOWS_AREA_HEIGHT_PX

// --- Nowe definicje dla paska tytulowego ---
#define RSSI_OFFSET_FROM_RIGHT  18
#define RSSI_FIELD_WIDTH        50
#define MAIN_TITLE_WIDTH        (SCREEN_WIDTH_ROTATED - RSSI_FIELD_WIDTH - RSSI_OFFSET_FROM_RIGHT)

// --- Stale kolory ---
// Uzyj #undef aby uniknac ostrzezen o przedefiniowaniu, a nastepnie zdefiniuj wlasne kolory
#ifdef ST77XX_GREEN
#undef ST77XX_GREEN
#endif
#ifdef ST77XX_YELLOW
#undef ST77XX_YELLOW
#endif
#ifdef ST77XX_BLUE
#undef ST77XX_BLUE
#endif
#ifdef ST77XX_RED
#undef ST77XX_RED
#endif

#define ST77XX_BLACK            0x0000 // Zdefiniowane dla pewnosci
#define ST77XX_WHITE            0xFFFF // Zdefiniowane dla pewnosci
#define ST77XX_DARKGREY         0x7BEF
#define ST77XX_LIGHTGREY        0xC618
#define ST77XX_BLUE_DARK        0x0010
#define ST77XX_RED_DARK         0x8000
#define ST77XX_BLUE_CALIBRATION 0x001F // Uzyjemy tego do kalibracji
#define ST77XX_GREEN            0x03E0
#define ST77XX_ORANGE_LIGHT     0xFD20
#define ST77XX_BLUE             0x001F
#define ST77XX_RED              0xF800

// --- Dodatkowy kolor szary dla trybu symulacji ---
#define ST77XX_SIMULATION_GREY  0x7BEF // Uzyjemy DARKGREY jako "szary" dla symulacji

enum DataInfo {
    Real,
    Simul
};
struct temperature {
    float temperature;      // Glowna temperatura (np. analogowa lub dominujaca)  
    enum DataInfo source;
};
struct height
{
	int RealHeght;
	enum DataInfo source;
};
struct angle
{
	int RealAngle;
	enum DataInfo source;
};
struct voltage
{
	float RealVoltage;
	enum DataInfo source;
};
struct rssi
{
	int RealRssi;
	enum DataInfo source;
};
struct isCalibrating
{
	bool RealCalibrate;
    char TextValue[32];
    int CalibrationConuter;
	enum DataInfo Source;
};
struct TempSource {
    char AnalogDigital;      // Glowna temperatura (np. analogowa lub dominujaca)  
    enum DataInfo source;
    int AnalogStatus;
    int DigitalStatus;
};
struct StatusSensor
{
    char state;
    enum DataInfo source;
};


// --- Nowa struktura do przekazywania danych do funkcji LCD_PUT ---
struct DisplayData {
    struct temperature MainTemperature;     // Glowna temperatura (np. analogowa lub dominujaca)
    struct TempSource MainTempeSource;      // tu przechowywujemy typ czyunka pomiaroweg analogowy lub cyfrowy
    struct height MainHeight;             // Wysokosc (z BMP280, jesli kalibrowany na wysokosc)
    struct angle MainAngle;              // Kat (z IMU)
    struct voltage MainVoltage;          // Napiecie z wejscia analogowego (fotorezystora)
    int brightnessPercent;  // Procent jasnosci podswietlenia LCD
    struct rssi MainRssi;               // Sila sygnalu RSSI BLE
    struct isCalibrating MainCalibrate;    // Flaga stanu kalibracji
    struct StatusSensor Wentylator; //// DODANO: 'W' dla wlaczonego wentylatora, 'n' dla wylaczonego
    struct StatusSensor Alarm;      //'A' dla alarmu, 'N' dla braku alarmu
  
    float rollAngle;         // DODANO: Kat Roll z IMU
    float pitchAngle;        // DODANO: Kat Pitch z IMU
    bool popUpWindow;        //wpisuje extra text na ekrane
    char textPopup[64];      //extra text
  
};

// --- Obiekt TFT - zewnetrzna deklaracja ---
extern Adafruit_ST7789 tft;

// --- Bufory pozaekranowe - zewnetrzne deklaracje ---
extern GFXcanvas16 tempWindowBufferGFX;
extern GFXcanvas16 heightWindowBufferGFX;
extern GFXcanvas16 angleWindowBufferGFX;
extern GFXcanvas16 graphBufferGFX;

// --- Zmienne globalne dla pomiaru jasnosci i regulacji podswietlenia ---
extern uint8_t currentBacklightPWM; // Aktualny poziom PWM podswietlenia (0-255)
extern float currentLightVoltage;    // Aktualne napiecie z fotorezystora
extern int testRssi; // Przykladowa wartosc RSSI, jesli chcesz ja kontrolowac z zewnatrz

// --- Prototypy funkcji ---
void LCD_init();
void LCD_PUT(const DisplayData* data);
void setBacklightPWM(uint8_t pwmValue);
void measureBrightness();

#endif // LCD_DISPLAY_H
