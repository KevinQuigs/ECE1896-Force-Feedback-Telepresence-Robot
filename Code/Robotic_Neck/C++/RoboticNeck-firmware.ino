
#include "NeckConfig.h"

//This is the configuration file, main structure in _main.ino
//CONFIGURATION SETTINGS:
#define COMMUNICATION COMM_SERIAL //Which communication protocol to use
//serial over USB
  #define SERIAL_BAUD_RATE 115200
  
//serial over Bluetooth
  #define BTSERIAL_DEVICE_NAME "RoboticNeck_ServoController" //Name of the bluetooth device


#define USING_CALIB_PIN false //When PIN_CALIB is shorted (or it's button pushed) it will reset calibration if this is on.


//PINS CONFIGURATION 
#if defined(ESP32)
  #define PIN_YAW_MOTOR 13
  #define PIN_PITCH_MOTOR 12
  #define PIN_ROLL_MOTOR 14

#elif defined(__AVR__)
  #define PIN_YAW_MOTOR 
  #define PIN_PITCH_MOTOR 3
  #define PIN_ROLL_MOTOR 4
#endif