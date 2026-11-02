Altivio is a microcontroller-based electronic height measuring system built around an ESP32 microcontroller and a VL53L1X Time-of-Flight distance sensor mounted at a calibrated reference height. During measurement, the sensor determines the distance from the overhead reference point to the top of the user's head. The controller calculates height, filters repeated readings, applies a stored calibration offset, and shows the result on an OLED display. The prototype is designed for rechargeable operation and can optionally transmit measurement data over
Bluetooth/Wi-Fi for logging or integration with a mobile application.

**Altivio** uses an overhead sensor mounted on a rigid vertical frame at a known reference height _($H_{ref}$)_ above the ground. When a user stands beneath the sensor, it samples the direct distance ($d$) to the apex of the head.  
$$\text{Height } (H) = H_{ref} - d + C_{cal}$$$H_{ref}$: **Calibrated sensor-to-floor datum plane distance.** 
$d$: **Measured sensor-to-head optical distance.**
$C_{cal}$: **Stored non-volatile systematic calibration offset.**

**Target Specifications**
Measurement Range:_ 30 cm – 400 cm_ (expandable up to 400 cm with VL53L1X configuration).   
Accuracy Target: Within $\pm 5\text{ mm}$ under controlled indoor posture. 
Display Units: _Centimeters (cm) with optional Imperial (inches) conversion and data-hold status. _

**Prerequisites**
Arduino IDE (version 2.0+ recommended)   
ESP32 Board Support Package installed via Arduino Board Manager ([https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json](https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json))  
Required Libraries: _Wire (Standard Arduino I²C library)_
Adafruit_SSD1306 & Adafruit_GFXPololu VL53L1X or SparkFun VL53L1X

**Firmware Flashing**
1) **Clone the repository**:_git clone [https://github.com/<your-username>/Altivio.git](https://github.com/jahinsabur/Altivio-Firmware/edit/main/README.md)
cd Altivio/firmware/Altivio_Firmware_

2) Open Altivio_Firmware.ino in Arduino IDE.  

3) Select your target board: Tools > Board > ESP32 Arduino > ESP32-C3 Dev Module.

4) Configure upload speed and USB CDC options appropriate for your board.
In config.h, set your baseline vertical mount height:
_C++#define BASELINE_HEIGHT_MM  2100.0  // Distance from ground to sensor aperture in mm
#define SYSTEM_OFFSET_MM    0.0     // Calibration offset_

5) Connect via USB-B and hit Upload.  

Calibration Procedure: 
Measure the exact vertical distance between the floor datum and the VL53L1X front optical window using a precision steel tape. Store this value in firmware as $H_{ref}$.  Place a planar calibration gauge of known height (e.g., 1000 mm block) directly beneath the field of view. Calculate systematic bias:$$C_{cal} = H_{actual} - (H_{ref} - d_{measured})$$Update $C_{cal}$ in EEPROM/Preferences to retain the offset across power cycles.   

Contributing & License Contributions, issues, and feature requests are welcome. Please open an issue to discuss design changes before submitting pull requests. 

Distributed under the MIT License. See LICENSE for more information.
