#include "AdvancedConfig.h"

//CONFIGURATION SETTINGS:
#define COMMUNICATION COMM_SERIAL 
#define SERIAL_BAUD_RATE 115200

//ANALOG INPUT CONFIG
#define FLIP_POTS  false  //Flip values from potentiometers (for fingers!) if they are backwards
//Ring and pinky backwards

#define USING_CALIB_PIN true //When PIN_CALIB is shorted (or it's button pushed) it will reset calibration if this is on.

#define USING_FORCE_FEEDBACK true //Force feedback haptics allow you to feel the solid objects you hold
#define SERVO_SCALING true //Dynamic scaling of servo motors

#if defined(ESP32)
  //(This configuration is for ESP32 DOIT V1 so make sure to change if you're on another board)
  #define PIN_PINKY     14
  #define PIN_RING      12
  #define PIN_MIDDLE    13
  #define PIN_INDEX     35
  #define PIN_THUMB     34
  #define PIN_CALIB     5 //button for recalibration (You can set this to GPIO0 to use the BOOT button, but only when using Bluetooth.)
  #define DEBUG_LED     2
  #define PIN_PINKY_MOTOR     32  //used for force feedback
  #define PIN_RING_MOTOR      33 //^
  #define PIN_MIDDLE_MOTOR    25 //^
  #define PIN_INDEX_MOTOR     26 //^
  #define PIN_THUMB_MOTOR     27 //^
#endif

