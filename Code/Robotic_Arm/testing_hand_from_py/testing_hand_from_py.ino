#include <Arduino.h>
#include <ESP32Servo.h>

// Servo pins
const int thumbPin = 15;
const int indexPin = 2;
const int middlePin = 4;
const int ringPin = 16;
const int pinkyPin = 17;
const int wristFlexPin = 5;    // wrist flexion/extension
const int wristRotatePin = 19; // wrist rotation

// Servo objects
Servo thumbF, indexF, middleF, ringF, pinkyF, wristFlex, wristRotate;

const int NUM_SERVOS = 7;
Servo* servos[NUM_SERVOS] = {&thumbF, &indexF, &middleF, &ringF, &pinkyF, &wristFlex, &wristRotate};

int servoPins[NUM_SERVOS] = {thumbPin, indexPin, middlePin, ringPin, pinkyPin, wristFlexPin, wristRotatePin};

// Tags that match what the GUI sends
const char* tags[NUM_SERVOS] = {"T", "I", "M", "R", "P", "WF", "WR"};





void setup() {
  Serial.begin(115200);

  // Attach servos with min/max pulse width
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i]->attach(servoPins[i], 500, 2400);
  }

  // Initialize servos to default positions
  thumbF.write(0);         
  indexF.write(0);           
  middleF.write(0);          
  ringF.write(180);          
  pinkyF.write(180);         
  wristFlex.write(50);       
  wristRotate.write(150);    

  Serial.println("ESP32 Ready");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int pos = 0;

      while (pos < input.length()) {
        bool matched = false;

        for (int s = 0; s < NUM_SERVOS; s++) {
          int tagLen = strlen(tags[s]);

          // Fingers (indices 0-4) need F prefix
          if (s <= 4) {
            if (input.substring(pos, pos + 1 + tagLen) == String("F") + tags[s]) {
              pos += 1 + tagLen;
              int angleStart = pos;

              
              while (pos < input.length() && isDigit(input[pos])) pos++;
                int angle = input.substring(angleStart, pos).toInt();

              // Call individual function
              switch (s) {
                case 0: moveThumb(angle); break;
                case 1: moveIndex(angle); break;
                case 2: moveMiddle(angle); break;
                case 3: moveRing(angle); break;
                case 4: movePinky(angle); break;
              }

              matched = true;
              break;
            }
          }
          // Wrist (indices 5-6) — no F
          else {
            if (input.substring(pos, pos + tagLen) == tags[s]) {
              pos += tagLen;
              int angleStart = pos;
              while (pos < input.length() && isDigit(input[pos])) pos++;
              int angle = input.substring(angleStart, pos).toInt();

              if (s == 5) moveWristFlex(angle);
              if (s == 6) moveWristRotate(angle);

              matched = true;
              break;
            }
          }
        }

        if (!matched) pos++;  // skip unknown chars
      }
    }
  }
}




void moveThumb(int angle) {
  angle = constrain(angle, 0, 60);  // optional safety limit
  thumbF.write(angle);
  Serial.print("Thumb -> "); Serial.println(angle);
}

void moveIndex(int angle) {
  angle = constrain(angle, 0, 160);
  indexF.write(angle);
  Serial.print("Index -> "); Serial.println(angle);
}

void moveMiddle(int angle) {
  angle = constrain(angle, 0, 160);
  middleF.write(angle);
  Serial.print("Middle -> "); Serial.println(angle);
}

void moveRing(int angle) {
  angle = constrain(angle, 20, 180);
  ringF.write(180 - angle);
  Serial.print("Ring -> "); Serial.println(angle);
}

void movePinky(int angle) {
  angle = constrain(angle, 20, 180);
  pinkyF.write(180 - angle);
  Serial.print("Pinky -> "); Serial.println(angle);
}

void moveWristFlex(int angle) {
  angle = constrain(angle, 10, 160);
  wristFlex.write(angle);
  Serial.print("WristFlex -> "); Serial.println(angle);
}

void moveWristRotate(int angle) {
  angle = constrain(angle, 10, 170);
  wristRotate.write(angle);
  Serial.print("WristRotate -> "); Serial.println(angle);
}