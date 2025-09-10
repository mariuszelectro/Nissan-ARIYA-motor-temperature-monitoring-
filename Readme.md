# Nissan Ariya Electric Motor Temperature Monitor
[![Kup mi kawę](https://img.buymeacoffee.com/button-api/?text=Buy%20me%20a%20coffee&emoji=&slug=mariuszelectro&button_colour=FFDD00&font_colour=000000&font_family=Poppins&outline_colour=000000&coffee_colour=ffffff)](https://www.buymeacoffee.com/mariuszelectro)

This project aims to create an independent system to monitor the electric motor's temperature in a Nissan Ariya. The system is designed to prevent sudden vehicle shutdowns caused by overheating.
This project was created as hobby not for comercial  use with maximise effect, not method.

---
### Project Background

The Nissan Ariya has a serious software flaw that can lead to dangerous situations. While driving on an incline, the motor's temperature can rise rapidly, and the car's system will cut power without warning. This can cause the vehicle to stop suddenly on a sloped road in the midle, creating a significant road safety hazard. Since there are no off-the-shelf monitoring systems available to address this problem, I've decided to build one myself.

---
### Key Features

* **Dual Temperature Sensors:** The system uses two diffrent sensors for precise temperature measurement. First Analg PT100, second digital DS18B20
* **Wireless Data Transmission:** Temperature status is sent wirelessly in real-time, trough BLE
* **Temperature Trend Analysis:** The software analyzes the rate of temperature increase, not just the current temperature level, which helps predict potential overheating events.
* **Fast Data Refresh:** Status information is refreshed and analyzed in less than one second.
* **Receiver Unit:** The receiver is equipped with a 1.69" LCD, a buzzer for alerts, a barometric pressure sensor to display altitude, and an inclinometer to show the road's tilt.
* **Receiver function:** The receiver present tempeperature as osciloscope digram for easy understand trends, alitude, and road's tilt.
* **Smart Functions:** A photoresistor automatically adjusts the display's brightness and can also be used as a button to silence the alarm.
* **Calibrator:** Optional Third party used for calibrate sensors.
* **Custom Housing:** All enclosures were designed using 3D modeling and are printed with MJF technology using Nylon for durability.
* **Cost-Effective:** Most of the necessary components can be purchased online at low prices.
* **AI-Powered Code:** A significant portion of the code was developed with the assistance of Gemini AI, but tested and coreted with me.
* **My contact:** mariuszelectro@gmail.com

