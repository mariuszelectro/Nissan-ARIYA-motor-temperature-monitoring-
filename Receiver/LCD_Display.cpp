#include "LCD_Display.h"
#include <cstdio> // Dla snprintf
#include <cstring> // Dla funkcji memmove
#include <cmath>   // Dla roundf

// --- Obiekty TFT i bufory - Definicje ---
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 tempWindowBufferGFX(TEMP_WINDOW_WIDTH_PX, TEMP_WINDOW_HEIGHT_PX);
GFXcanvas16 heightWindowBufferGFX(HEIGHT_WINDOW_WIDTH_PX, HEIGHT_WINDOW_HEIGHT_PX);
GFXcanvas16 angleWindowBufferGFX(ANGLE_WINDOW_WIDTH_PX, ANGLE_WINDOW_HEIGHT_PX);
GFXcanvas16 graphBufferGFX(SCREEN_WIDTH_ROTATED, GRAPH_AREA_HEIGHT_PX);
// Bufory do przechowywania poprzednich wartosci stringow dla optymalizacji
static char lastTempString[10] = "        ";
static char lastHeightString[10] = "        ";
static char lastAngleString[10] = "        ";
static char lastMainTitleString[30];
// Wymaga odswiezenia, gdy zmienia sie kalibracja
static char lastRssiString[10];
static char lastAlarmChar = ' ';
static char lastWentylatorChar = ' ';
static uint16_t lastMainTitleBgColor = 0x2104;
static unsigned long lastBlinkTime = 0;
static bool isBlinkingRed = false;


// --- Zmienne globalne dla pomiaru jasnosci i regulacji podswietlenia (definiowane tutaj) ---
float currentLightVoltage = 0.0;
uint8_t currentBacklightPWM = 0;

// --- Zmienna dla testowej wartosci RSSI ---
int testRssi = 60;
// Globalna zmienna, dostepna z zewnatrz
static uint16_t currentRssiBgColor = ST77XX_BLUE_DARK;
// --- Funkcja do ustawiania jasnosci podswietlenia ---
void setBacklightPWM(uint8_t pwmValue) {
    if (pwmValue != currentBacklightPWM) {
        analogWrite(TFT_BL, pwmValue);
        currentBacklightPWM = pwmValue;
        Serial.print("Ustawiono jasnosc podswietlenia na PWM: ");
        Serial.println(pwmValue);
    }
}

// --- Funkcja do pomiaru jasnosci i regulacji podswietlenia ---
// --- Funkcja inicjalizacji LCD ---
void LCD_init() {
    tft.init(ST7789_PHYSICAL_WIDTH, ST7789_PHYSICAL_HEIGHT);
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);

    pinMode(TFT_BL, OUTPUT);
    setBacklightPWM(255);
    // Poczatkowo pelna jasnosc

    uint8_t vscrdef_params[6];
    vscrdef_params[0] = (TFA_SCROLL_PARAM >> 8);
    vscrdef_params[1] = (TFA_SCROLL_PARAM & 0xFF);
    vscrdef_params[2] = ((ST7789_PHYSICAL_HEIGHT - TFA_SCROLL_PARAM - BFA_SCROLL_PARAM) >> 8);
    vscrdef_params[3] = ((ST7789_PHYSICAL_HEIGHT - TFA_SCROLL_PARAM - BFA_SCROLL_PARAM) & 0xFF);
    vscrdef_params[4] = (BFA_SCROLL_PARAM >> 8);
    vscrdef_params[5] = (BFA_SCROLL_PARAM & 0xFF);
    tft.sendCommand(ST77XX_VSCRDEF, vscrdef_params, 6);
    // Initial fills - to make sure areas are drawn initially
    tft.fillRect(0, TITLE_Y_START_ON_SCREEN, MAIN_TITLE_WIDTH, TITLE_AREA_HEIGHT_PX, 0x2104);
    tft.fillRect(SCREEN_WIDTH_ROTATED - RSSI_OFFSET_FROM_RIGHT - RSSI_FIELD_WIDTH, TITLE_Y_START_ON_SCREEN, RSSI_FIELD_WIDTH, TITLE_AREA_HEIGHT_PX, ST77XX_BLUE_DARK);
    tft.fillRect(0, GRAPH_Y_START_ON_SCREEN, SCREEN_WIDTH_ROTATED, GRAPH_AREA_HEIGHT_PX, ST77XX_BLACK);
    tft.fillRect(0, SEPARATOR_Y_START_ON_SCREEN, SCREEN_WIDTH_ROTATED, SEPARATOR_HEIGHT_PX, ST77XX_BLACK);
    tft.fillRect(0, TEST_WINDOWS_Y_START_ON_SCREEN, SCREEN_WIDTH_ROTATED, TEST_WINDOWS_AREA_HEIGHT_PX, ST77XX_BLACK);

    graphBufferGFX.fillScreen(ST77XX_BLACK);

    tempWindowBufferGFX.setTextWrap(false);
    tempWindowBufferGFX.setTextColor(ST77XX_WHITE);
    tempWindowBufferGFX.setFont(&FreeSansBold24pt7b);

    heightWindowBufferGFX.setFont(NULL);
    heightWindowBufferGFX.setTextWrap(false);
    heightWindowBufferGFX.setTextColor(ST77XX_WHITE);

    angleWindowBufferGFX.setFont(NULL);
    angleWindowBufferGFX.setTextWrap(false);
    angleWindowBufferGFX.setTextColor(ST77XX_WHITE);

    tft.setFont(NULL);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
}

// --- Funkcja aktualizacji LCD ---
void LCD_PUT(const DisplayData* data) {
    unsigned long start_time = micros();
    unsigned long currentTime = millis();
    // --- 1. Obsluga przewijania oscylogramu ---
    uint16_t* bufferPtr = graphBufferGFX.getBuffer();
    for (int y = 0; y < GRAPH_AREA_HEIGHT_PX; y++) {
        memmove(bufferPtr + (y * SCREEN_WIDTH_ROTATED),
                bufferPtr + (y * SCREEN_WIDTH_ROTATED) + 1,
                (SCREEN_WIDTH_ROTATED - 1) * sizeof(uint16_t));
    }
    graphBufferGFX.drawFastVLine(SCREEN_WIDTH_ROTATED - 1, 0, GRAPH_AREA_HEIGHT_PX, ST77XX_BLACK);
    
    // --- Rysowanie słupków (POPRAWIONY BLOK) ---
    int tempBarHeight = 0;
    uint16_t tempBarColor = ST77XX_BLACK;
    
    // Logika do obliczania wysokosci i koloru slupka
    if (data->MainTemperature.temperature != -1) {
        if (data->MainTemperature.temperature < 20.0) {
            tempBarColor = ST77XX_BLUE;
            tempBarHeight = 1;
        } else if (data->MainTemperature.temperature >= 20.0 && data->MainTemperature.temperature < 80.0) {
            tempBarColor = ST77XX_GREEN;
        } else if (data->MainTemperature.temperature >= 80.0 && data->MainTemperature.temperature <= 99.0) {
            tempBarColor = ST77XX_ORANGE_LIGHT;
        } else {
            tempBarColor = ST77XX_RED;
        }
        
        if (data->MainTemperature.temperature >= 20.0) {
            float clampedTemp = constrain(data->MainTemperature.temperature, 20.0f, 100.0f);
            tempBarHeight = (int)((clampedTemp - 20.0f) * (GRAPH_AREA_HEIGHT_PX / 80.0f));
            tempBarHeight = constrain(tempBarHeight, 0, GRAPH_AREA_HEIGHT_PX);
            // KLUCZOWA KOREKTA: Gwarantuje minimalną wysokość 1px w zakresie skalowania
            if (data->MainTemperature.temperature >= 20.0 && tempBarHeight == 0) {
                 tempBarHeight = 1; 
            }
        }
    } else {
        tempBarColor = ST77XX_BLACK;
        tempBarHeight = 0;
    }
    
    // Rysowanie slupka z bialym pikselem na szczycie
    if (tempBarHeight > 0) {
        if (tempBarHeight >= 2) {
            graphBufferGFX.drawFastVLine(SCREEN_WIDTH_ROTATED - 1, GRAPH_AREA_HEIGHT_PX - tempBarHeight + 1, tempBarHeight - 1, tempBarColor);
            graphBufferGFX.drawPixel(SCREEN_WIDTH_ROTATED - 1, GRAPH_AREA_HEIGHT_PX - tempBarHeight, ST77XX_WHITE);
        } else if (tempBarHeight == 1) {
            graphBufferGFX.drawPixel(SCREEN_WIDTH_ROTATED - 1, GRAPH_AREA_HEIGHT_PX - 1, tempBarColor);
        }
    }
    
    // === NOWY BLOK ===
    // Sprawdzamy, czy wlaczony jest tryb kalibracji ORAZ czy tekst ma sensowna wartosc
    if (data->MainCalibrate.RealCalibrate && data->MainCalibrate.TextValue[0] != 0) {
        // Ustawienie czcionki uzywanej dla temperatury
        graphBufferGFX.setFont(&FreeSansBold18pt7b);
    
        // Ustawienie koloru tekstu na podstawie zrodla danych
        uint16_t textColor = ST77XX_WHITE;
        if (data->MainCalibrate.Source == Simul) {
            textColor = ST77XX_SIMULATION_GREY;
        }
        graphBufferGFX.setTextColor(textColor);
    
        // Obliczenie wymiarow tekstu, zeby go wycentrowac
        int16_t x1, y1;
        uint16_t text_w, text_h;
        graphBufferGFX.getTextBounds(data->MainCalibrate.TextValue, 0, 0, &x1, &y1, &text_w, &text_h);
    
        // Obliczenie pozycji dla tekstu (teraz dokładnie w srodku okna)
        int16_t x_pos = (SCREEN_WIDTH_ROTATED / 2) - (text_w / 2);
        int16_t y_pos = (GRAPH_AREA_HEIGHT_PX / 2) - (text_h / 2);
        
        // Rysowanie tekstu na istniejącym wykresie
        graphBufferGFX.setCursor(x_pos, y_pos);
        graphBufferGFX.print(data->MainCalibrate.TextValue);
    
    }
    // === KONIEC NOWEGO BLOKU ===
    // =====obsluga  extra textu =====//
    if (data->popUpWindow==true) {
        // Ustawienie czcionki uzywanej dla temperatury
        graphBufferGFX.setFont(&FreeSansBold18pt7b);
    
        // Ustawienie koloru tekstu na podstawie zrodla danych
        uint16_t textColor = ST77XX_WHITE;
        graphBufferGFX.setTextColor(textColor);
        // Obliczenie wymiarow tekstu, zeby go wycentrowac
        int16_t x1, y1;
        uint16_t text_w, text_h;
        graphBufferGFX.getTextBounds(data->textPopup, 0, 0, &x1, &y1, &text_w, &text_h);
        // Obliczenie pozycji dla tekstu (teraz dokładnie w srodku okna)
        int16_t x_pos = (SCREEN_WIDTH_ROTATED / 2) - (text_w / 2);
        int16_t y_pos = (GRAPH_AREA_HEIGHT_PX / 2) - (text_h / 2);
        // Rysowanie tekstu na istniejącym wykresie
        graphBufferGFX.setCursor(x_pos, y_pos);
        graphBufferGFX.print(data->textPopup);
    }
    // === KONIEC NOWEGO BLOKU ===

    //graphBufferGFX.drawFastVLine(SCREEN_WIDTH_ROTATED - 1, GRAPH_AREA_HEIGHT_PX - tempBarHeight, tempBarHeight, tempBarColor);
    // --- 2. Wizualizacja danych pomiarowych (dolny obszar BFA) ---

    // --- A. Temperatura ---
    char newTempIntPart[6];
    char newTempDecPart[3];
    uint16_t tempWindowBgColor = ST77XX_BLACK; // Domyślne tło

    if (data->MainTemperature.temperature != -1) {
        int intPart = (int)data->MainTemperature.temperature;
        int decPart = (int)( roundf( (data->MainTemperature.temperature - intPart) * 10 ) );
        if (decPart == 10) {
            intPart++;
            decPart = 0;
        }

        snprintf(newTempIntPart, sizeof(newTempIntPart), "%3d", intPart);
        snprintf(newTempDecPart, sizeof(newTempDecPart), ",%1d", decPart);
        if (data->MainTemperature.temperature < 20.0) tempWindowBgColor = ST77XX_BLUE;
        else if (data->MainTemperature.temperature >= 20.0 && data->MainTemperature.temperature< 80.0) tempWindowBgColor = ST77XX_GREEN;
        else if (data->MainTemperature.temperature >= 80.0 && data->MainTemperature.temperature <= 99.0) tempWindowBgColor = ST77XX_ORANGE_LIGHT;
        else tempWindowBgColor = ST77XX_RED;
    } else {
        snprintf(newTempIntPart, sizeof(newTempIntPart), "---");
        snprintf(newTempDecPart, sizeof(newTempDecPart), ",-");
        tempWindowBgColor = ST77XX_BLACK;
    }

    char currentFullTempString[10];
    snprintf(currentFullTempString, sizeof(currentFullTempString), "%s%s", newTempIntPart, newTempDecPart);

    bool tempChanged = (strcmp(currentFullTempString, lastTempString) != 0);
    static uint16_t lastTempBgColor = ST77XX_BLACK;
    bool tempBgColorChanged = (tempWindowBgColor != lastTempBgColor);
    static enum DataInfo lastTempSource = Simul;
    bool tempSourceChanged = (data->MainTemperature.source != lastTempSource);


    if (tempChanged || tempBgColorChanged || tempSourceChanged) {
        memcpy(lastTempString, currentFullTempString, sizeof(currentFullTempString));
        lastTempBgColor = tempWindowBgColor;
        lastTempSource = data->MainTemperature.source; // Zaktualizuj stan zrodla

        tempWindowBufferGFX.fillScreen(tempWindowBgColor);

        // --- Ustawienie koloru czcionki na podstawie pola 'source' ---
        uint16_t tempTextColor;
        switch (data->MainTemperature.source) {
            case Real:
                tempTextColor = ST77XX_WHITE;
                break;
            case Simul:
                tempTextColor = ST77XX_SIMULATION_GREY;
                break;
        }

        // Ustawianie koloru dla litery zrodla i całej temperatury
        tempWindowBufferGFX.setTextColor(tempTextColor);
        
        // Wyświetlanie litery zrodla
        tempWindowBufferGFX.setFont(NULL); // Użycie domyślnej czcionki
        tempWindowBufferGFX.setTextSize(2);
        
        char sourceLetter = ' '; // Domyślnie spacja, żeby nic się nie wyświetlało
        if (data->MainTempeSource.AnalogDigital == 'A' || data->MainTempeSource.AnalogDigital == 'C') {
            sourceLetter = data->MainTempeSource.AnalogDigital;
        }
        tempWindowBufferGFX.setCursor(3, 3); // Dostosowana pozycja dla wiekszej czcionki
        tempWindowBufferGFX.print(sourceLetter);
        
        // Wyświetlanie temperatury
        tempWindowBufferGFX.setFont(&FreeSansBold24pt7b);
        tempWindowBufferGFX.setTextSize(1);

        int16_t x1_int, y1_int, x1_dec, y1_dec, x1_unit, y1_unit;
        uint16_t w_int, h_int, w_dec, h_dec, w_unit, h_unit;

        tempWindowBufferGFX.getTextBounds(newTempIntPart, 0, 0, &x1_int, &y1_int, &w_int, &h_int);
        int baseline_y_main = (TEMP_WINDOW_HEIGHT_PX - h_int) / 2 - y1_int + 10;
        int temp_x_pos_main = 25;

        tempWindowBufferGFX.setCursor(temp_x_pos_main, baseline_y_main);
        tempWindowBufferGFX.print(newTempIntPart);

        tempWindowBufferGFX.setFont(&FreeSansBold18pt7b);

        tempWindowBufferGFX.getTextBounds(newTempDecPart, 0, 0, &x1_dec, &y1_dec, &w_dec, &h_dec);

        int decimal_y_pos = baseline_y_main;
        tempWindowBufferGFX.setCursor(temp_x_pos_main + w_int, decimal_y_pos);
        tempWindowBufferGFX.print(newTempDecPart);

        const char* unit_label_oc = "oC";
        tempWindowBufferGFX.setFont(&FreeSansBold12pt7b);
        tempWindowBufferGFX.setTextColor(ST77XX_LIGHTGREY);
        // Jednostka moze byc zawsze LightGrey
        tempWindowBufferGFX.getTextBounds(unit_label_oc, 0, 0, &x1_unit, &y1_unit, &w_unit, &h_unit);
        tempWindowBufferGFX.setCursor(TEMP_WINDOW_WIDTH_PX - w_unit - 2, 2 - y1_unit);
        if (data->MainTemperature.temperature != -1) {
            tempWindowBufferGFX.print(unit_label_oc);
        }
    }


    // --- B. Wysokosc ---
    char newHeightString[10];
    uint16_t heightWindowBgColor;
    uint16_t heightTextColor; 

    if (data->MainCalibrate.RealCalibrate) {
        snprintf(newHeightString, sizeof(newHeightString), "----");
        heightWindowBgColor = ST77XX_BLUE_CALIBRATION;
        // Kalibracja - tlo niebieskie
        heightTextColor = ST77XX_GREEN;
        // Kalibracja - tekst zielony
    } else {
        if (data->MainHeight.RealHeght == -1) {
            snprintf(newHeightString, sizeof(newHeightString), "----");
        } else {
            snprintf(newHeightString, sizeof(newHeightString), "%4d", data->MainHeight.RealHeght);
        }
        heightWindowBgColor = ST77XX_BLACK;
        // Normalny tryb - tlo czarne
        
        // Ustawianie koloru tekstu dla wysokosci na podstawie pola source
        switch (data->MainHeight.source) {
            case Real:
                heightTextColor = ST77XX_WHITE;
                break;
            case Simul:
                heightTextColor = ST77XX_SIMULATION_GREY;
                break;
        }
    }

    bool heightChanged = (strcmp(newHeightString, lastHeightString) != 0);
    static uint16_t lastHeightBgColor = ST77XX_BLACK;
    bool heightBgColorChanged = (heightWindowBgColor != lastHeightBgColor);
    static bool lastIsCalibratingHeight = false;
    bool isCalibratingHeightChanged = (data->MainCalibrate.RealCalibrate != lastIsCalibratingHeight);
    static enum DataInfo lastHeightSource = Simul;
    bool heightSourceChanged = (data->MainHeight.source != lastHeightSource);
    if (heightChanged || heightBgColorChanged || isCalibratingHeightChanged || heightSourceChanged) {
        memcpy(lastHeightString, newHeightString, sizeof(newHeightString));
        lastHeightBgColor = heightWindowBgColor;
        lastIsCalibratingHeight = data->MainCalibrate.RealCalibrate;
        lastHeightSource = data->MainHeight.source;

        heightWindowBufferGFX.fillScreen(heightWindowBgColor);
        heightWindowBufferGFX.setTextColor(heightTextColor); 
        heightWindowBufferGFX.setFont(NULL);
        heightWindowBufferGFX.setTextSize(3);

        int16_t x1, y1;
        uint16_t w, h;
        heightWindowBufferGFX.getTextBounds(newHeightString, 0, 0, &x1, &y1, &w, &h);

        int height_angle_offset_y = 5;
        heightWindowBufferGFX.setCursor((HEIGHT_WINDOW_WIDTH_PX - w) / 2 - x1, (HEIGHT_WINDOW_HEIGHT_PX - h) / 2 - y1 + height_angle_offset_y);
        heightWindowBufferGFX.print(newHeightString);
        const char* unit_height = "m";
        heightWindowBufferGFX.setTextSize(2);
        heightWindowBufferGFX.setTextColor(ST77XX_LIGHTGREY); 
        heightWindowBufferGFX.getTextBounds(unit_height, 0, 0, &x1, &y1, &w, &h);
        heightWindowBufferGFX.setCursor(HEIGHT_WINDOW_WIDTH_PX - w - 2, 2 - y1 + height_angle_offset_y);
        if (!data->MainCalibrate.RealCalibrate) {
            heightWindowBufferGFX.print(unit_height);
        }
    }

    // --- C. Kat ---
    char newAngleString[10];
    uint16_t angleWindowBgColor;
    uint16_t angleTextColor; 

    if (data->MainCalibrate.RealCalibrate) {
        snprintf(newAngleString, sizeof(newAngleString), "---");
        angleWindowBgColor = ST77XX_BLUE_CALIBRATION;
        // Kalibracja - tlo niebieskie
        angleTextColor = ST77XX_GREEN;
        // Kalibracja - tekst zielony
    } else {
        if (data->MainAngle.Valid == false) {
            snprintf(newAngleString, sizeof(newAngleString), "---");
        } else {
            snprintf(newAngleString, sizeof(newAngleString), "%2d", data->MainAngle.RealAngle);
        }
        angleWindowBgColor = ST77XX_BLACK;
        // Normalny tryb - tlo czarne
        
        // Ustawianie koloru tekstu dla kata na podstawie pola source
        switch (data->MainAngle.source) {
            case Real:
                angleTextColor = ST77XX_WHITE;
                break;
            case Simul:
                angleTextColor = ST77XX_SIMULATION_GREY;
                break;
        }
    }

    bool angleChanged = (strcmp(newAngleString, lastAngleString) != 0);
    static uint16_t lastAngleBgColor = ST77XX_BLACK;
    bool angleBgColorChanged = (angleWindowBgColor != lastAngleBgColor);
    static bool lastIsCalibratingAngle = false;
    bool isCalibratingAngleChanged = (data->MainCalibrate.RealCalibrate != lastIsCalibratingAngle);
    static enum DataInfo lastAngleSource = Simul;
    bool angleSourceChanged = (data->MainAngle.source != lastAngleSource);
    if (angleChanged || angleBgColorChanged || isCalibratingAngleChanged || angleSourceChanged) {
        memcpy(lastAngleString, newAngleString, sizeof(newAngleString));
        lastAngleBgColor = angleWindowBgColor;
        lastIsCalibratingAngle = data->MainCalibrate.RealCalibrate;
        lastAngleSource = data->MainAngle.source;

        angleWindowBufferGFX.fillScreen(angleWindowBgColor);
        angleWindowBufferGFX.setTextColor(angleTextColor); 
        angleWindowBufferGFX.setFont(NULL);
        angleWindowBufferGFX.setTextSize(3);

        int16_t x1, y1;
        uint16_t w, h;
        angleWindowBufferGFX.getTextBounds(newAngleString, 0, 0, &x1, &y1, &w, &h);

        int height_angle_offset_y = 5;
        angleWindowBufferGFX.setCursor((ANGLE_WINDOW_WIDTH_PX - w) / 2 - x1, (ANGLE_WINDOW_HEIGHT_PX - h) / 2 - y1 + height_angle_offset_y);
        angleWindowBufferGFX.print(newAngleString);
        const char* unit_angle = "%";
        angleWindowBufferGFX.setTextSize(2);
        angleWindowBufferGFX.setTextColor(ST77XX_LIGHTGREY); 
        angleWindowBufferGFX.getTextBounds(unit_angle, 0, 0, &x1, &y1, &w, &h);
        angleWindowBufferGFX.setCursor(ANGLE_WINDOW_WIDTH_PX - w - 2, 2 - y1 + height_angle_offset_y);
        if (!data->MainCalibrate.RealCalibrate) {
            angleWindowBufferGFX.print(unit_angle);
        }
    }

    // --- Wyswietlanie napiecia i jasnosci w procentach / napisu KALIBRACJA ---
    char currentMainTitleString[30];
    uint16_t mainTitleBgColor = 0x2104; // Nowy, ciemniejszy szary
    uint16_t mainTitleTextColor; 

    // Logika mrugania kolorem paska
    if (data->Alarm.state == 'A') {
        if (currentTime - lastBlinkTime >= 1000) {
            isBlinkingRed = !isBlinkingRed;
            lastBlinkTime = currentTime;
        }
        mainTitleBgColor = isBlinkingRed ? ST77XX_RED : 0x2104;
    } else {
        isBlinkingRed = false; // Resetujemy stan
    }

    if (data->MainCalibrate.RealCalibrate) {
        snprintf(currentMainTitleString, sizeof(currentMainTitleString), "KALIBRACJA");
        mainTitleBgColor = ST77XX_BLUE_CALIBRATION; 
    } else {
        snprintf(currentMainTitleString, sizeof(currentMainTitleString), "%.2fV %d%%", data->MainVoltage.RealVoltage, data->brightnessPercent);
        // Kolor paska zmieniany wyżej, nie tutaj
    }

    // Ustawianie koloru glownego tytulu na podstawie WYLACZNIE MainVoltage.source
    switch (data->MainVoltage.source) {
        case Real:
            mainTitleTextColor = ST77XX_WHITE;
            break;
        case Simul:
            mainTitleTextColor = ST77XX_SIMULATION_GREY;
            break;
    }

    bool mainTitleChanged = (strcmp(currentMainTitleString, lastMainTitleString) != 0);
    static enum DataInfo lastMainVoltageSource = Simul;
    bool mainVoltageSourceChanged = (data->MainVoltage.source != lastMainVoltageSource);
    
    // Zmiany dla alarmu i wentylatora
    bool alarmCharChanged = (data->Alarm.state != lastAlarmChar);
    bool wentylatorCharChanged = (data->Wentylator.state != lastWentylatorChar);
    bool mainTitleBgColorChanged = (mainTitleBgColor != lastMainTitleBgColor);

    if (mainTitleChanged || mainTitleBgColorChanged || mainVoltageSourceChanged || alarmCharChanged || wentylatorCharChanged) {
        memcpy(lastMainTitleString, currentMainTitleString, sizeof(currentMainTitleString));
        lastMainTitleBgColor = mainTitleBgColor;
        lastMainVoltageSource = data->MainVoltage.source;
        lastAlarmChar = data->Alarm.state;
        lastWentylatorChar = data->Wentylator.state;

        // ---- Rysowanie tla paska tytulowego w poprawnej szerokosci ----
        int mainTitleBarWidth = SCREEN_WIDTH_ROTATED - RSSI_FIELD_WIDTH - RSSI_OFFSET_FROM_RIGHT;
        tft.fillRect(0, TITLE_Y_START_ON_SCREEN, mainTitleBarWidth, TITLE_AREA_HEIGHT_PX, mainTitleBgColor);
        tft.setFont(NULL);
        tft.setTextSize(2);
        
        // ---- Ustawianie kolorow i rysowanie napiecia i jasnosci ----
        uint16_t mainTitleTextColor;
        switch (data->MainVoltage.source) {
            case Real:
                mainTitleTextColor = ST77XX_WHITE;
                break;
            case Simul:
                mainTitleTextColor = ST77XX_SIMULATION_GREY;
                break;
        }
        tft.setTextColor(mainTitleTextColor);

        int16_t x1, y1;
        uint16_t w, h;
        tft.getTextBounds(currentMainTitleString, 0, 0, &x1, &y1, &w, &h);
        int currentMainTitle_x = MAIN_TITLE_WIDTH / 2 - w / 2;
        int currentMainTitle_y = TITLE_Y_START_ON_SCREEN + (TITLE_AREA_HEIGHT_PX - h) / 2;
        tft.setCursor(currentMainTitle_x, currentMainTitle_y);
        tft.print(currentMainTitleString);
        
        // --- Wyswietlanie ikon A i W ----
        int currentXPosition = currentMainTitle_x + w;
        int iconWidth = 20;
        int charWidth = 10;
        int charHeight = 12;

        // --- Rysowanie ikony alarmu 'A' ---
        int alarmIconX = currentXPosition + 5;
        // Zawsze rysuj prostokąt, zmieniaj tylko kolor
        uint16_t alarmRectColor = (data->Alarm.state == 'A') ? ST77XX_RED : mainTitleBgColor;
        tft.fillRect(alarmIconX, TITLE_Y_START_ON_SCREEN, iconWidth, TITLE_AREA_HEIGHT_PX, alarmRectColor);
        
        uint16_t alarmCharColor;
        switch (data->Alarm.source) {
            case Real:
                alarmCharColor = ST77XX_WHITE;
                break;
            case Simul:
                alarmCharColor = ST77XX_SIMULATION_GREY;
                break;
        }
        tft.setTextColor(alarmCharColor);
        tft.setCursor(alarmIconX + (iconWidth - charWidth) / 2, currentMainTitle_y);
        tft.print(data->Alarm.state == 'A' ? 'A' : ' ');
        
        // --- Rysowanie ikony wentylatora 'W' ---
        int wentylatorIconX = alarmIconX + iconWidth + 5;
        // Zawsze rysuj prostokąt, zmieniaj tylko kolor
        uint16_t wentylatorRectColor = (data->Wentylator.state == 'W') ? ST77XX_ORANGE : mainTitleBgColor;
        tft.fillRect(wentylatorIconX, TITLE_Y_START_ON_SCREEN, iconWidth, TITLE_AREA_HEIGHT_PX, wentylatorRectColor);

        uint16_t wentylatorCharColor;
        switch (data->Wentylator.source) {
            case Real:
                wentylatorCharColor = ST77XX_WHITE;
                break;
            case Simul:
                wentylatorCharColor = ST77XX_SIMULATION_GREY;
                break;
        }
        tft.setTextColor(wentylatorCharColor);
        tft.setCursor(wentylatorIconX + (iconWidth - charWidth) / 2, currentMainTitle_y);
        tft.print(data->Wentylator.state == 'W' ? 'W' : ' ');
    }
    
    // --- Wyswietlanie RSSI w prawej czesci obszaru tytulu ---
    char currentRssiString_display[10];
    uint16_t newRssiBgColor;
    uint16_t rssiTextColor; 

    if (data->MainRssi.RealRssi == -1) {
        snprintf(currentRssiString_display, sizeof(currentRssiString_display), " -- ");
        newRssiBgColor = ST77XX_RED_DARK;
    } else {
        snprintf(currentRssiString_display, sizeof(currentRssiString_display), "%3d", data->MainRssi.RealRssi);
        newRssiBgColor = ST77XX_BLUE_DARK;
    }
    
    // Ustawianie koloru tekstu dla RSSI na podstawie pola source
    switch (data->MainRssi.source) {
        case Real:
            rssiTextColor = ST77XX_WHITE;
            break;
        case Simul:
            rssiTextColor = ST77XX_SIMULATION_GREY;
            break;
    }

    bool rssiChanged = (strcmp(currentRssiString_display, lastRssiString) != 0);
    bool bgColorChanged = (newRssiBgColor != currentRssiBgColor);
    static enum DataInfo lastRssiSource = Simul;
    bool rssiSourceChanged = (data->MainRssi.source != lastRssiSource);
    if (rssiChanged || bgColorChanged || rssiSourceChanged) {
        memcpy(lastRssiString, currentRssiString_display, sizeof(currentRssiString_display));
        currentRssiBgColor = newRssiBgColor;
        lastRssiSource = data->MainRssi.source; 

        tft.fillRect(SCREEN_WIDTH_ROTATED - RSSI_OFFSET_FROM_RIGHT - RSSI_FIELD_WIDTH, TITLE_Y_START_ON_SCREEN, RSSI_FIELD_WIDTH, TITLE_AREA_HEIGHT_PX, currentRssiBgColor);
        tft.setFont(NULL);
        tft.setTextSize(2);
        tft.setTextColor(rssiTextColor); 

        int16_t x1, y1;
        uint16_t w, h;
        tft.getTextBounds(currentRssiString_display, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor(SCREEN_WIDTH_ROTATED - RSSI_OFFSET_FROM_RIGHT - RSSI_FIELD_WIDTH + (RSSI_FIELD_WIDTH - w) / 2,
                        TITLE_Y_START_ON_SCREEN + (TITLE_AREA_HEIGHT_PX - h) / 2);
        tft.print(currentRssiString_display);
    }

    unsigned long mid_time = micros();
    // --- Rysowanie na LCD z buforow i aktualizacje obszarow ---
    tft.drawRGBBitmap(0, GRAPH_Y_START_ON_SCREEN, graphBufferGFX.getBuffer(), SCREEN_WIDTH_ROTATED, GRAPH_AREA_HEIGHT_PX);
    if (tempChanged || tempBgColorChanged || tempSourceChanged) {
        tft.drawRGBBitmap(0, TEST_WINDOWS_Y_START_ON_SCREEN, tempWindowBufferGFX.getBuffer(), TEMP_WINDOW_WIDTH_PX, TEMP_WINDOW_HEIGHT_PX);
    }
    if (heightChanged || heightBgColorChanged || isCalibratingHeightChanged || heightSourceChanged) {
        tft.drawRGBBitmap(TEMP_WINDOW_WIDTH_PX, TEST_WINDOWS_Y_START_ON_SCREEN, heightWindowBufferGFX.getBuffer(), HEIGHT_WINDOW_WIDTH_PX, HEIGHT_WINDOW_HEIGHT_PX);
    }
    if (angleChanged || angleBgColorChanged || isCalibratingAngleChanged || angleSourceChanged) {
        tft.drawRGBBitmap(TEMP_WINDOW_WIDTH_PX + HEIGHT_WINDOW_WIDTH_PX, TEST_WINDOWS_Y_START_ON_SCREEN, angleWindowBufferGFX.getBuffer(), ANGLE_WINDOW_WIDTH_PX, ANGLE_WINDOW_HEIGHT_PX);
    }

    unsigned long end_time = micros();

    // === ZMODYFIKOWANY KOD ===
    Serial.printf("LCD time %lu/%lu/%lu us)\n", mid_time - start_time, end_time - mid_time, end_time - start_time);
    // === KONIEC ZMODYFIKOWANEGO KODU ===
}