#include <Arduino.h>
#include <ESP32Servo.h>

// Servo pins
const int thumbPin = 15;
const int indexPin = 2;
const int middlePin = 4;
const int ringPin = 16;
const int pinkyPin = 17;
const int wristFlexPin = 5;   // wrist flexion/extension
const int wristRotatePin = 18;  // wrist rotation

Servo thumbF, indexF, middleF, ringF, pinkyF, wristFlex, wristRotate;

String input = "";
int initial_angle = 0;
int angle = 180;

void setup() {
  // Serial.begin(115200);
  thumbF.attach(thumbPin, 500, 2400);
  indexF.attach(indexPin, 500, 2400);
  middleF.attach(middlePin, 500, 2400);
  ringF.attach(ringPin, 500, 2400);
  pinkyF.attach(pinkyPin, 500, 2400);
  wristFlex.attach(wristFlexPin, 500, 2400);
  wristRotate.attach(wristRotatePin, 500, 2400);
  
  //Serial.println("Initializing servos to 0°...");

  // Initialize all angles to 180 (extended out)
  thumbF.write(initial_angle);        
  indexF.write(initial_angle);       
  middleF.write(initial_angle);       
  ringF.write(180 - initial_angle);       
  pinkyF.write(180 - initial_angle);      
  wristFlex.write(0);    
  wristRotate.write(0);  
  delay(500);

}

void loop() {

    if (initial_angle <= 60) {
      thumbF.write(initial_angle);   
      delay(500);
    }
    indexF.write(initial_angle);      delay(500);
    middleF.write(initial_angle);      delay(500);
    ringF.write(180 - initial_angle);   delay(500);
    pinkyF.write(180 - initial_angle);  

    if (initial_angle == angle)
      initial_angle = 0; 
    else
      initial_angle += 10;   
    
    delay(500);

                     
}