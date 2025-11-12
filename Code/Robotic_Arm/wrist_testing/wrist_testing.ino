#include <Arduino.h>
#include <ESP32Servo.h>

// Servo pins
const int thumbPin = 15;
const int indexPin = 2;
const int middlePin = 4;
const int ringPin = 16;
const int pinkyPin = 17;
const int wristFlexPin = 5;   // wrist flexion/extension
const int wristRotatePin = 19;  // wrist rotation

Servo thumbF, indexF, middleF, ringF, pinkyF, wristFlex, wristRotate;

String input = "";
int initial_angle = 90;
int finger_angle = 0;
int angle = 0;
const int MAX_ANGLE = 160;
const int MIN_ANGLE = 20;
bool down = true;

void setup() {
  Serial.begin(115200);     // ✅ ENABLED SERIAL OUTPUT
  delay(200);               // small delay so serial monitor can sync

  thumbF.attach(thumbPin, 500, 2400);
  indexF.attach(indexPin, 500, 2400);
  middleF.attach(middlePin, 500, 2400);
  ringF.attach(ringPin, 500, 2400);
  pinkyF.attach(pinkyPin, 500, 2400);
  wristFlex.attach(wristFlexPin, 500, 2400);
  wristRotate.attach(wristRotatePin, 500, 2400);

  // Initialize rotation servo
  thumbF.write(finger_angle);        
  indexF.write(finger_angle);       
  middleF.write(finger_angle);       
  ringF.write(180 - finger_angle);       
  pinkyF.write(180 - finger_angle);  
  wristFlex.write(initial_angle);
  wristRotate.write(initial_angle);
  delay(500);

  Serial.println("Setup complete. Beginning loop...");
}

void loop() {

    Serial.print("Angle: ");
    Serial.println(initial_angle);

    wristRotate.write(initial_angle);

    if (initial_angle <= MIN_ANGLE)
      down = false; 
    else if (initial_angle >= MAX_ANGLE)
      down = true;

    if (down)
      initial_angle -= 10;  
    else
      initial_angle += 10; 

    delay(500);
}
