# Teensy-Data-Logger
Wi-Fi Connected Teensy Data Logger with GPS and Accelerometer

This is a basic implementation of a Wi-Fi connected data logger.  I used the Teensy 3.1 as the Adafruit Wi-Fi module
requires more SRAM than my Arduino can spare.  I also designed a carrier board for this project and shared both the EAGLE
file as well as the board on OSHPark.  Complete project documentation can be found here: http://triembed.org/blog/?p=959

The device will connect to Wifi, get a GPS fix, and send location information to Ubidots when it detects movement.  
