Altivio is a microcontroller-based electronic height measuring system built
around an ESP32 microcontroller and a VL53L1X Time-of-Flight distance sensor mounted at a calibrated
reference height on a rigid vertical frame. During measurement, the sensor determines the distance
from the overhead reference point to the top of the user's head. The controller calculates height, filters
repeated readings, applies a stored calibration offset, and shows the result on an OLED display. The
prototype is designed for rechargeable operation and can optionally transmit measurement data over
Bluetooth/Wi-Fi for logging or integration with a mobile application.
