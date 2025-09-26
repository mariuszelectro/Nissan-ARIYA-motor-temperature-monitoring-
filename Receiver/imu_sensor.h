#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <LSM6DS3.h>


/* Flters Params
#define Q_angle_set 0.0005          //lower value = more stable bult higer latency 0.001
#define Q_bias_set 0.003            //gyro dift
#define R_measure_set 0.3           //How trust accel higer= trust more gyro 0.03

#define R_measure_high_set 0.8      //0,5 more = moer trust gyro 0,03-1,2
#define R_measure_low_set 0.09      //0,03 sesivity when no acceelration to  higer= more stable
#define GYRO_THRESHOLD_set  0.8     //1.0 accereartion limit to usec for calculation  less= trust gyro 
#define ACCEL_TOLERANCE_set 0.3      //0.2  huher = more smouth
#define ACCEL_LPF_ALPHA_set 0.05      //lower value = more filtererd 0,2
*/
//* Filters Params - Optymalizacja pod maksymalną stabilność w samochodzie *//

#define KalmmaPresets 2

#if (KalmmaPresets==1)
#define Q_angle_set 0.0005     // Szum procesu dla kąta. Mniejsza wartość = większa stabilność i mniejsza czułość na ruch.
// Zakres: 0.0001 - 0.01 (niższe wartości dla stabilnych platform jak samochód, wyższe dla dynamicznych robotów)
#define Q_bias_set 0.003       // Szum procesu dla dryfu żyroskopu.
// Zakres: 0.001 - 0.01 (niższe wartości dla czułych sensorów, wyższe dla gorszej jakości)
#define R_measure_set 3.0      // Szum pomiaru z akcelerometru. Kluczowy parametr! Im wyższa wartość, tym bardziej
// ignorujesz dane z akcelerometru i ufasz żyroskopowi. To minimalizuje wpływ hamowania i przyspieszania.
// Zakres: 0.5 - 5.0 (niższe wartości dla bardziej zaufanych pomiarów, wyższe dla bardzo zaszumionych)
#define R_measure_high_set 5.0     // Szum pomiaru używany podczas gwałtownych ruchów (powyżej progu GYRO_THRESHOLD_set).
// Zakres: 2.0 - 10.0 (powinna być wyższa niż R_measure_set, aby wstrząsy były mocno odrzucane)
#define R_measure_low_set 0.20     // Szum pomiaru gdy nie ma przyspieszenia. Używany do lekkiego wygładzenia.
// Zakres: 0.03 - 0.5 (niższe wartości dla większej czułości w spoczynku, wyższe dla stabilniejszego odczytu)
#define GYRO_THRESHOLD_set 2.0     // Próg wykrywania gwałtownych ruchów (w stopniach/s). Po przekroczeniu tej wartości,
// filtr przechodzi na ustawienia R_measure_high_set, ufając bardziej żyroskopowi.
// Zakres: 0.5 - 5.0 (niższe wartości dla szybszego reagowania, wyższe dla ignorowania małych wstrząsów)
#define ACCEL_TOLERANCE_set 0.8    // Wygładzanie surowych danych z akcelerometru. Wyższa wartość = większe wygładzenie.
// Zakres: 0.1 - 1.0 (zazwyczaj 0.2-0.8)
#define ACCEL_LPF_ALPHA_set 0.02   // Współczynnik filtra dolnoprzepustowego (LPF) dla akcelerometru. Mniejsza wartość
// = mocniejsze filtrowanie, które wprowadza większe opóźnienie. Idealne do odrzucania szumu wibracji.
// Zakres: 0.01 - 0.5 (niższe wartości do filtrowania silnych wibracji, wyższe dla szybszej reakcji)
#elif (KalmmaPresets==2)
/* Filters Params - Ustawienia dla MAKSYMALNEJ STABILNOŚCI i powolnej reakcji w samochodzie */
#define Q_angle_set 0.0001     // Szum procesu dla kąta. Ekstremalnie niska wartość, aby maksymalnie "usztywnić" i wygładzić odczyt.
// Zakres: 0.0001 - 0.01 (najniższa wartość dla maksymalnej stabilności)
#define Q_bias_set 0.003       // Szum procesu dla dryfu żyroskopu. Wartość standardowa, zwykle nie wymaga zmian.
// Zakres: 0.001 - 0.01 
#define R_measure_set 20.0     // Domyślny szum pomiaru. Ustawiony wysoko jako wartość bazowa.
// Zakres: 0.5 - 25.0
#define R_measure_high_set 25.0 // Kluczowy parametr! Bardzo wysoka wartość, aby filtr niemal CAŁKOWICIE ignorował akcelerometr
// podczas przyspieszania, hamowania i na wybojach.
// Zakres: 2.0 - 30.0 (wyższe wartości = mniejsza wrażliwość na wstrząsy)
#define R_measure_low_set 0.15 // Szum pomiaru w stanie spoczynku. Dość niska wartość, aby umożliwić bardzo powolną
// korektę odczytu, gdy auto stoi lub jedzie idealnie płynnie.
// Zakres: 0.03 - 0.5 
#define GYRO_THRESHOLD_set 1.5 // Próg wykrywania ruchu. Obniżony, aby filtr szybciej przełączał się w tryb "ignorowania
// akcelerometru" nawet przy delikatnych manewrach.
// Zakres: 0.5 - 5.0 
#define ACCEL_TOLERANCE_set 0.1 // Kluczowy parametr! Bardzo niska tolerancja. Nawet najmniejsze przyspieszenie (większe niż 0.1g)
// zostanie wykryte i spowoduje odrzucenie danych z akcelerometru.
// Zakres: 0.1 - 1.0 (dla Twojego celu wymagana jest niska wartość)
#define ACCEL_LPF_ALPHA_set 0.015 // Współczynnik filtra LPF. Bardzo niska wartość, aby ekstremalnie filtrować wibracje
// z silnika i nierówności drogi, zanim dane trafią do filtru Kalmana. Wprowadzi to celowe opóźnienie.
// Zakres: 0.01 - 0.5 (im niższa wartość, tym silniejsze filtrowanie i większy lag)
#endif

/**
 * @brief Struktura do przechowywania odczytów z IMU.
 */
struct IMUData {
    float tilt;        // Nachylenie wzdłuż osi Y (do przodu/tyłu)
    float angle;       // Kąt wzdłuż osi X (na boki)
    float kalmanAngle; // Estymowany kąt z filtru Kalmana
    int samplesCollected; // Liczba zebranych próbek od ostatniego odczytu
};

/**
 * @brief Inicjalizuje czujnik IMU.
 * @return true jesli inicjalizacja powiodla sie, false w przeciwnym razie.
 */
bool imu_init();

/**
 * @brief Kalibruje czujnik IMU. Musi być wywoływana w pętli.
 * @return true jeśli kalibracja została zakończona, false w przeciwnym razie.
 */
bool Call_IMU_Calibration(int write);

/**
 * @brief Odczytuje uśrednione dane z IMU i zwraca obliczone kąty.
 * @param data Wskaznik do struktury IMUData, ktora zostanie wypelniona danymi.
 * @return true jesli odczyt powiodl sie i dane sa poprawne, false w przeciwnym razie.
 */
bool readIMUData(IMUData* data);

/**
 * @brief Zbieranie danych i aktualizacja filtru Kalmana.
 * Nalezy wywolywac w petli glownej.
 */
void CollectImu(void);

bool LoadImuCalibration(void);
void setCallIMUasZero(void);

#endif