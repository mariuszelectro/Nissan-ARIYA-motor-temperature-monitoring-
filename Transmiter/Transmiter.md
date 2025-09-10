
# Nissan Ariya - Advanced Electric Motor Temperature Monitor

This project aims to create an independent, intelligent system for monitoring and actively managing the electric motor's temperature in a Nissan Ariya. The primary goal is to prevent sudden, dangerous vehicle shutdowns caused by overheating.

---

### Transmitter Assumptions and Functions

The transmitter is the core of the system, focusing on advanced data analysis and control that goes beyond simple temperature readings.

#### Inputs and Power
* **Inputs:** The transmitter has three inputs:
    1.  A dedicated input for a **PT100** sensor.
    2.  An input for a digital **DS18B20** sensor.
    3.  A power input ranging from **12-16V** from the car's on-board network.
* **Power Management:** A comparator-based circuit monitors the 12V battery voltage. The system activates at **13.6V** and enters a low-power sleep mode at **12.9V** (with hysteresis), consuming only 3µA in standby mode and 30mA in opertional mode 

#### Measurement Circuitry
* **PT100 Measurement:** The input circuit consists of a Wheatstone bridge with three **105R resistors** (0.1% tolerance, low ppm). This specific resistance value was chosen because the project focuses on temperatures **above 15°C**. The bridge is powered by a ~2mA current source, and the resulting voltage signal is amplified by an instrumental amplifier.
* **Signal Conversion:** The voltage is converted to a **0-12V** level and then fed through a voltage divider to the measurement input of a **XIAO nRF52840** module.
* **Data Processing:** The XIAO module samples the voltage every few milliseconds, applies filtering and advanced averaging, and then converts the voltage value into a temperature reading. It also handles the digital DS18B20 sensor.

---

### Sensor Selection: Why Two?

Although using a single, digital **DS18B20** sensor would simplify the circuit, past experience shows it's susceptible to interference from strong magnetic fields, which causes it to **freeze or hang**. To ensure reliability and continuous measurements, a redundant sensor setup with two different types of sensors was implemented. **By cycling the power, the system can restore the sensor to operation.** The system intelligently selects the proper, available sensor by analyzing both signals.

---

### Advanced Logic and Control
* **Sensor Selection:** The system analyzes data from both sensors, always selecting the one that is available and has a higher temperature reading.
* **Trend Analysis:** The software analyzes the temperature's upward trend, predicting its increase over a **30-second period**.
* **Early Warning:** An alarm is activated well in advance if the trend analysis predicts the temperature will reach the critical limit of **70°C**.
* **Actuator Control:** An output is available to control an external actuator, such as an **additional cooling fan**. The fan's activation is also based on the temperature trend analysis.
* **Analog Output:** An analog output signal (0-12V) is provided for diagnostic purposes, where **10V corresponds to 100°C**.
* **Control Method:** The fan can be controlled via a hardware comparator or by software.

#### Communication and Housing
* **BLE Beacon Mode:** The XIAO module operates as a BLE "beacon," transmitting measurement data, alarm flags, fan status, and a running counter as dynamic advertising data. This allows it to be tracked with a standard Bluetooth scanner on a phone.
* **Debug Mode:** When connected, the module enters a debug mode (SSP BLE profile ), sending data to the Bluefruit Connect application to generate plots on canvas, from both sensors.
* **Antenna:** The custom PCB version includes a modified antenna layout with a u.FL connector for an external antenna, providing better range than the factory one.
* **Enclosure:** The housing is 3D printed using MJF technology with black nylon powder, resulting in a very solid and durable enclosure with 3mm thick walls, labyrinthine seals, and cable holders.
* **Availability:** The PCB schematic and layout will be published soon. While others can print the model using different technologies, the precise fit and accuracy of the MJF process may not be achievable.
