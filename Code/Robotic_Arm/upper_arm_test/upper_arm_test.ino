#include <Arduino.h>

// ======================= JOINT DEFINITIONS =======================
const int shoulderLatPot = 25;
const int shoulderLatServo1 = 26;
const int shoulderLatServo2 = 27;

const int shoulderFrontPot = 14;
const int shoulderFrontServo1 = 12;
const int shoulderFrontServo2 = 13;

const int shoulderTwistPot = 4;
const int shoulderTwistServo1 = 18;
const int shoulderTwistServo2 = 19;

const int bicepPot = 2;
// const int bicepServo1 = 16;
// const int bicepServo2 = 17;

const int bicepServo1 = 12;
const int bicepServo2 = 13;

// ======================= POT LIMITS =======================
int shoulderLatMin = 2050;
int shoulderLatMax = 2500;

int shoulderFrontMin = 200;
int shoulderFrontMax = 3000;

int shoulderTwistMin = 500;
int shoulderTwistMax = 3500;

int bicepMin = 10;
int bicepMax = 1500;

// ======================= MOVEMENT HELPERS =======================
void moveLateralUp() { int p = analogRead(shoulderLatPot); if (p < shoulderLatMax) { digitalWrite(shoulderLatServo1,HIGH); digitalWrite(shoulderLatServo2,LOW); } else stopLateral(); }
void moveLateralDown() { int p = analogRead(shoulderLatPot); if (p > shoulderLatMin) { digitalWrite(shoulderLatServo1,LOW); digitalWrite(shoulderLatServo2,HIGH); } else stopLateral(); }
void stopLateral() { digitalWrite(shoulderLatServo1,LOW); digitalWrite(shoulderLatServo2,LOW); }

void moveFrontUp() { int p = analogRead(shoulderFrontPot); if (p < shoulderFrontMax) { digitalWrite(shoulderFrontServo1,HIGH); digitalWrite(shoulderFrontServo2,LOW); } else stopFront(); }
void moveFrontDown() { int p = analogRead(shoulderFrontPot); if (p > shoulderFrontMin) { digitalWrite(shoulderFrontServo1,LOW); digitalWrite(shoulderFrontServo2,HIGH); } else stopFront(); }
void stopFront() { digitalWrite(shoulderFrontServo1,LOW); digitalWrite(shoulderFrontServo2,LOW); }

void moveTwistRight() { int p = analogRead(shoulderTwistPot); if (p < shoulderTwistMax) { digitalWrite(shoulderTwistServo1,HIGH); digitalWrite(shoulderTwistServo2,LOW); } else stopTwist(); }
void moveTwistLeft() { int p = analogRead(shoulderTwistPot); if (p > shoulderTwistMin) { digitalWrite(shoulderTwistServo1,LOW); digitalWrite(shoulderTwistServo2,HIGH); } else stopTwist(); }
void stopTwist() { digitalWrite(shoulderTwistServo1,LOW); digitalWrite(shoulderTwistServo2,LOW); }

void moveBicepUp() { int p = analogRead(bicepPot); if (p < bicepMax) { digitalWrite(bicepServo1,HIGH); digitalWrite(bicepServo2,LOW); } else stopBicep(); }
void moveBicepDown() { int p = analogRead(bicepPot); if (p > bicepMin) { digitalWrite(bicepServo1,LOW); digitalWrite(bicepServo2,HIGH); } else stopBicep(); }
void stopBicep() { digitalWrite(bicepServo1,LOW); digitalWrite(bicepServo2,LOW); }

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);

  pinMode(shoulderLatPot, INPUT);
  pinMode(shoulderFrontPot, INPUT);
  pinMode(shoulderTwistPot, INPUT);
  pinMode(bicepPot, INPUT);

  pinMode(shoulderLatServo1, OUTPUT);
  pinMode(shoulderLatServo2, OUTPUT);
  pinMode(shoulderFrontServo1, OUTPUT);
  pinMode(shoulderFrontServo2, OUTPUT);
  pinMode(shoulderTwistServo1, OUTPUT);
  pinMode(shoulderTwistServo2, OUTPUT);
  pinMode(bicepServo1, OUTPUT);
  pinMode(bicepServo2, OUTPUT);

  stopLateral(); stopFront(); stopTwist(); stopBicep();

  Serial.println("Arm system initialized.");
  Serial.println("Send commands: lateral_up, lateral_down, front_up, front_down, twist_left, twist_right, elbow_up, elbow_down, stop_all");
}

// ======================= LOOP =======================
void loop() {
  // Read Serial input
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // remove whitespace

    if (cmd == "lateral_up") moveLateralUp();
    else if (cmd == "lateral_down") moveLateralDown();
    else if (cmd == "front_up") moveFrontUp();
    else if (cmd == "front_down") moveFrontDown();
    else if (cmd == "twist_left") moveTwistLeft();
    else if (cmd == "twist_right") moveTwistRight();
    else if (cmd == "elbow_up") moveBicepUp();
    else if (cmd == "elbow_down") moveBicepDown();
    else if (cmd == "stop_all") { stopLateral(); stopFront(); stopTwist(); stopBicep(); }
    else Serial.println("Unknown command");
  }

  // Optionally print pot values for feedback
  int latVal = analogRead(shoulderLatPot);
  int frontVal = analogRead(shoulderFrontPot);
  int twistVal = analogRead(shoulderTwistPot);
  int bicepVal = analogRead(bicepPot);
  Serial.printf("Lat:%d | Front:%d | Twist:%d | Bicep:%d\n", latVal, frontVal, twistVal, bicepVal);

  delay(100);
}
