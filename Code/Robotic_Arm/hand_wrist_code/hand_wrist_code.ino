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

void setup() {
  Serial.begin(115200);
  thumbF.attach(thumbPin, 500, 2400);
  indexF.attach(indexPin, 500, 2400);
  middleF.attach(middlePin, 500, 2400);
  ringF.attach(ringPin, 500, 2400);
  pinkyF.attach(pinkyPin, 500, 2400);
  wristFlex.attach(wristFlexPin, 500, 2400);
  wristRotate.attach(wristRotatePin, 500, 2400);
  
  Serial.println("Initializing servos to 0°...");

  // Initialize all angles to 0
  thumbF.write(0);        delay(200);
  indexF.write(0);       delay(200);
  middleF.write(0);       delay(200);
  ringF.write(0);        delay(200);
  pinkyF.write(0);        delay(200);
  wristFlex.write(0);    delay(200);
  wristRotate.write(0);  delay(200);

  Serial.println("Ready. Example commands:");
  Serial.println("  thumb 60");
  Serial.println("  index 120");
  Serial.println("  wristRotate 45");
}

void loop() {
  // Read serial input until newline
  if (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        processCommand(input);
        input = "";
      }
    } else {
      input += c;
    }
  }
}

// --------------------------------------------------
// Process commands
// --------------------------------------------------
void processCommand(String cmd) {
  cmd.trim();  // remove spaces

  // Split command into two parts
  int spaceIndex = cmd.indexOf(' ');
  if (spaceIndex < 0) {
    Serial.println("Invalid command. Use: name angle");
    return;
  }

  String name = cmd.substring(0, spaceIndex);
  int angle = cmd.substring(spaceIndex + 1).toInt();

  // ---- Match servo by name ----
  Servo* targetServo = nullptr;

  if (name == "thumb")          targetServo = &thumbF;
  else if (name == "index")     targetServo = &indexF;
  else if (name == "middle")    targetServo = &middleF;
  else if (name == "ring")      targetServo = &ringF;
  else if (name == "pinky")     targetServo = &pinkyF;
  else if (name == "wristFlex") targetServo = &wristFlex;
  else if (name == "wristRotate") targetServo = &wristRotate;
  else {
    Serial.println("Unknown servo: " + name);
    return;
  }

  // ---- Move servo ----
  Serial.print("Moving ");
  Serial.print(name);
  Serial.print(" to ");
  Serial.print(angle);
  Serial.println("°");

  targetServo->write(angle);
}