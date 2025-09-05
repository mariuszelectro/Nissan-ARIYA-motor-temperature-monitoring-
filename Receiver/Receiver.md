

### **1. Main Module**

---

* **Name:** XIAO nRF52840 Sense
* **Key Features:**
    * Built-in 6-axis Inertial Measurement Unit (IMU) based on a gyroscope and accelerometer for motion and orientation measurement.
    * Bluetooth Low Energy (BLE) module for wireless communication with the transmitter.

---

### **2. Display**

---

* **Type:** 1.69-inch LCD display (based on the [Waveshare 1.69inch LCD Module](https://www.waveshare.com/wiki/1.69inch_LCD_Module))
* **Interface:** Communicates via an SPI (Serial Peripheral Interface) bus.
* **Function:** Used for visualizing data, such as sensor measurements, device status, or other project information.

---

### **3. Sensors**

---

* **Pressure and Temperature Sensor:**
    * **Name:** BMP280 (based on the [HiLetgo BMP280](https://www.amazon.com/HiLetgo-Precision-BMP280-3-3-Atmospheric-Pressure/dp/B01ICN5QPU?th=1))
    * **Characteristics:** A barometric sensor that measures atmospheric pressure for altitude calculation, i2c imtrface. Requires a version with two mounting holes for mechanical stabilization of the XIAO module within the enclosure.
    * **Function:** Used **exclusively** for measuring altitude based on pressure readings.

* **Photoresistor:**
    * **Name:** GL5537 (5537 series)
    * **Characteristics:** A light sensor whose resistance changes with light intensity. 5 mm diameter. ([Link to GL5537 photoresistor](https://juriedengineering.com/products/gl5537-photo-light-sensitive-resistor-photoresistor-55375mm))
    * **Function:** Used to measure ambient light levels.

---

### **4. Audio Components**

---

* **Buzzer:**
    * **Type:** A passive buzzer, converted to active.
    * **Characteristics:** 12mm x 7.5mm in size.
    * **Function:** Used to generate audio signals, alarms, or notifications.

---

### **5. Enclosure and Power Supply**

---

* **Type:** A two-part screw-fastened enclosure.
* **Production Technology:** Printed in **MJF** (Multi Jet Fusion) 3D technology.
* **Function:** Provides precise and secure mechanical mounting for all internal electronic components.
* **Mounting:**
    * **Options:** Magnetic or screw-based mounting.
    * **Location:** Car air vent holder or a standard magnetic mount.
* **Power Supply:** USB-C port.

---

### **6. Software Functionality**

---

* **BLE Communication:**
    * **Mode:** The device operates in **passive listener** (beacon passive listener) mode.
    * **Characteristics:** It does not establish an active Bluetooth (BT) connection in any profile. It receives broadcast data from an external measurement beacon.
    * **Received Data:**
        * Temperature from an analog sensor.
        * Temperature from a digital sensor.
        * Fan status.
        * Alarm status.
        * A control counter (ensures data is not duplicated).

* **User Interface (LCD Display):**
    * **Screen Structure:** Divided into several sections.
    * **Sections:**
        * **Title Bar:** Contains the title and **status flags** (e.g., connection status, alarm).
        * **RSSI Section:** Displays the signal strength (RSSI) of the transmitter.
        * **Temperature Oscillogram:** Graphical bars drawn on the screen corresponding to the measured temperature value. They automatically scroll to the left every second. This uses the **display controller's hardware mechanisms**.
        * **Temperature Value:** A numerical indication of the current temperature.
        * **Altitude:** Displays the current altitude.
        * **Road Incline Angle:** The road's incline angle value, shown in a percentage.
    * **Dynamic Backgrounds:** The oscillogram and temperature value sections have defined background colors that change depending on the measured temperature range.

* **Photoresistor Operation:**
    * **Function 1 (Brightness):** Measures external light intensity for automatic screen brightness adjustment (dark and light modes).
    * **Function 2 (Virtual Button):** Acts as a button for calibration. Activated by covering it for 10 seconds after the vehicle starts.
    * **Function 3 (Alarm):** Allows the user to silence an active alarm.
    * **Note:** All measurements for brightness and "presses" are filtered in software to prevent flickering or accidental triggers.

* **Buzzer Operation:**
    * **Characteristics:** Active and dynamic.
    * **Dependencies:** The tone and length of the generated sound depend on the type of alarm and any further temperature increase. The duration of each beep is variable.

* **Sensor Calibration:**
    * **Sensor Types:** Barometric and position (gyroscope/accelerometer).
    * **Calibration Methods:**
        1.  **Button Press:** Manual calibration activation.
        2.  **Signal Reception:** Automatic calibration upon receiving a signal from a dedicated calibration beacon.
    * **Calibration Process:**
        * **Inclinometer:** Zeros the incline angle readings to a zero position, regardless of the device's current orientation.
        * **Pressure:** Compensates absolute pressure for a fixed altitude of 270 meters above sea level.
    * **Calibration Availability:** The calibration function is available **only within a 10-second time window after the device is powered on**.
    * **Accuracy:** The altitude measurement has an accuracy of approximately 1 meter.
