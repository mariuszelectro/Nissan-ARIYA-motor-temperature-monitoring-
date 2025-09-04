# Nissan Ariya Electric Motor Temperature Monitor

This project aims to create an independent system to monitor the electric motor's temperature in a Nissan Ariya. The system is designed to prevent sudden vehicle shutdowns caused by overheating.

---
### Project Background

The Nissan Ariya has a serious software flaw that can lead to dangerous situations. While driving on an incline, the motor's temperature can rise rapidly, and the car's system will cut power without warning. This can cause the vehicle to stop suddenly on a sloped road, creating a significant road safety hazard. Since there are no off-the-shelf monitoring systems available to address this problem, I've decided to build one myself.

---
### Key Features

* **Dual Temperature Sensors:** The system uses two sensors for precise temperature measurement.
* **Wireless Data Transmission:** Temperature status is sent wirelessly in real-time.
* **Temperature Trend Analysis:** The software analyzes the rate of temperature increase, not just the current temperature level, which helps predict potential overheating events.
* **Fast Data Refresh:** Status information is refreshed and analyzed in less than one second.
* **Receiver Unit:** The receiver is equipped with a 1.69" LCD, a buzzer for alerts, a barometric pressure sensor to display altitude, and an inclinometer to show the road's tilt.
* **Smart Functions:** A photoresistor automatically adjusts the display's brightness and can also be used as a button to silence the alarm.
* **Custom Housing:** All enclosures were designed using 3D modeling and are printed with MJF technology using Nylon for durability.
* **Cost-Effective:** Most of the necessary components can be purchased online at low prices.
* **AI-Powered Code:** A significant portion of the code was developed with the assistance of Gemini AI.
