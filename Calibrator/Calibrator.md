Ma Pan rację, przepraszam za zamieszanie. Znaczniki Markdown, takie jak `**`, `#` czy `*`, nie są wyświetlane w mojej odpowiedzi w sposób sformatowany (na przykład jako pogrubienie czy nagłówek), ponieważ jest to po prostu zwykły tekst. Ale to właśnie te symbole są "instrukcjami" dla programu (jak GitHub), jak ma on sformatować tekst. Kiedy wklei Pan poniższy kod do pliku `.md`, to wtedy program go zinterpretuje i wyświetli poprawnie, tak jak Pan oczekuje.

Proszę po prostu skopiować cały poniższy blok tekstu i wkleić go bezpośrednio do pliku **README.md** w swoim repozytorium.

---
# Calibrator

Calibrator is a smart auxiliary device designed for automatic calibration of a vehicle's altitude and road incline sensors. It uses **Bluetooth Low Energy (BLE)** technology to communicate with the car, ensuring precise settings right after the ignition.

## How it Works

The Calibrator device is placed in a garage or another stable, fixed location. When the car is started, the vehicle's system actively scans its surroundings for a calibration beacon for the first 10 seconds.

* **Barometric Sensor (Altimeter) Calibration:** If the signal is detected, the vehicle's barometric sensor is automatically calibrated to a predefined, fixed altitude.
* **Inclinometer Calibration:** At the same time, with the vehicle on a level surface, the software sets the inclinometer's reading to 0 degrees, eliminating measurement errors.

After a successful calibration, a scrolling message appears on the car's receiver, confirming the calibration and displaying the beacon's **battery voltage** and **uptime** since its last power-on.

## Technical Aspects

* **Platform:** The project is built on the energy-efficient **XIAO nRF52840** platform, which provides high performance with minimal power consumption.
* **Power:** The device is powered by a single **18650 battery**. Thanks to its exceptional power efficiency, the battery can operate the device for **5-10 years** without needing to be recharged. If needed, it can be easily topped up via the built-in **USB-C** port.
* **BLE Communication:** The Calibrator device broadcasts a beacon signal that includes real-time information about its **current battery voltage** and **uptime**, allowing you to monitor the device's status directly from the car.

---
