#include <Arduino.h>

// ======================= JOINT DEFINITIONS =======================
// LATERAL RAISE (side lift)
const int shoulderLatPot = 25;
const int shoulderLatServo1 = 26; // H-bridge IN1
const int shoulderLatServo2 = 27; // H-bridge IN2

// FRONT RAISE (front lift)
const int shoulderFrontPot = 14;
const int shoulderFrontServo1 = 12;
const int shoulderFrontServo2 = 13;

// SHOULDER TWIST (rotation)
const int shoulderTwistPot = 4;
const int shoulderTwistServo1 = 18;
const int shoulderTwistServo2 = 19;

// BICEP (elbow flex)
const int bicepPot = 2;
const int bicepServo1 = 16;
const int bicepServo2 = 17;

// ======================= POT LIMITS (placeholder) =======================
int shoulderLatMin = 2050;
int shoulderLatMax = 2500;

int shoulderFrontMin = 200;
int shoulderFrontMax = 3000;

int shoulderTwistMin = 500;
int shoulderTwistMax = 3500;

int bicepMin = 10;
int bicepMax = 1500;

// ======================= TARGET VARIABLES =======================
float target_x, target_y, target_z;
float current_x, current_y, current_z;

// ======================= MOVEMENT HELPERS =======================
// All use H-bridge control logic: 
// Servo1 HIGH + Servo2 LOW = positive direction
// Servo1 LOW  + Servo2 HIGH = negative direction
// Both LOW = stop

// ---------- Lateral ----------
void moveLateralUp() {
  int potVal = analogRead(shoulderLatPot);
  if (potVal < shoulderLatMax) {
    digitalWrite(shoulderLatServo1, HIGH);
    digitalWrite(shoulderLatServo2, LOW);
  } else {
    digitalWrite(shoulderLatServo1, LOW);
    digitalWrite(shoulderLatServo2, LOW);
  }
}

void moveLateralDown() {
  int potVal = analogRead(shoulderLatPot);
  if (potVal > shoulderLatMin) {
    digitalWrite(shoulderLatServo1, LOW);
    digitalWrite(shoulderLatServo2, HIGH);
  } else {
    digitalWrite(shoulderLatServo1, LOW);
    digitalWrite(shoulderLatServo2, LOW);
  }
}

void stopLateral() {
  digitalWrite(shoulderLatServo1, LOW);
  digitalWrite(shoulderLatServo2, LOW);
}

// ---------- Front ----------
void moveFrontUp() {
  int potVal = analogRead(shoulderFrontPot);
  if (potVal < shoulderFrontMax) {
    digitalWrite(shoulderFrontServo1, HIGH);
    digitalWrite(shoulderFrontServo2, LOW);
  } else {
    stopFront();
  }
}

void moveFrontDown() {
  int potVal = analogRead(shoulderFrontPot);
  if (potVal > shoulderFrontMin) {
    digitalWrite(shoulderFrontServo1, LOW);
    digitalWrite(shoulderFrontServo2, HIGH);
  } else {
    stopFront();
  }
}

void stopFront() {
  digitalWrite(shoulderFrontServo1, LOW);
  digitalWrite(shoulderFrontServo2, LOW);
}

// ---------- Twist ----------
void moveTwistRight() {
  int potVal = analogRead(shoulderTwistPot);
  if (potVal < shoulderTwistMax) {
    digitalWrite(shoulderTwistServo1, HIGH);
    digitalWrite(shoulderTwistServo2, LOW);
  } else {
    stopTwist();
  }
}

void moveTwistLeft() {
  int potVal = analogRead(shoulderTwistPot);
  if (potVal > shoulderTwistMin) {
    digitalWrite(shoulderTwistServo1, LOW);
    digitalWrite(shoulderTwistServo2, HIGH);
  } else {
    stopTwist();
  }
}

void stopTwist() {
  digitalWrite(shoulderTwistServo1, LOW);
  digitalWrite(shoulderTwistServo2, LOW);
}

// ---------- Bicep ----------
void moveBicepUp() {
  int potVal = analogRead(bicepPot);
  if (potVal < bicepMax) {
    digitalWrite(bicepServo1, HIGH);
    digitalWrite(bicepServo2, LOW);
  } else {
    stopBicep();
  }
}

void moveBicepDown() {
  int potVal = analogRead(bicepPot);
  if (potVal > bicepMin) {
    digitalWrite(bicepServo1, LOW);
    digitalWrite(bicepServo2, HIGH);
  } else {
    stopBicep();
  }
}

void stopBicep() {
  digitalWrite(bicepServo1, LOW);
  digitalWrite(bicepServo2, LOW);
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);

  // Potentiometer inputs
  pinMode(shoulderLatPot, INPUT);
  pinMode(shoulderFrontPot, INPUT);
  pinMode(shoulderTwistPot, INPUT);
  pinMode(bicepPot, INPUT);

  // H-bridge outputs
  pinMode(shoulderLatServo1, OUTPUT);
  pinMode(shoulderLatServo2, OUTPUT);
  pinMode(shoulderFrontServo1, OUTPUT);
  pinMode(shoulderFrontServo2, OUTPUT);
  pinMode(shoulderTwistServo1, OUTPUT);
  pinMode(shoulderTwistServo2, OUTPUT);
  pinMode(bicepServo1, OUTPUT);
  pinMode(bicepServo2, OUTPUT);

  // Initialize all LOW
  stopLateral();
  stopFront();
  stopTwist();
  stopBicep();

  Serial.println("Arm system initialized.");
}

// ======================= MAIN LOOP =======================
void loop() {
  int latVal = analogRead(shoulderLatPot);
  int frontVal = analogRead(shoulderFrontPot);
  int twistVal = analogRead(shoulderTwistPot);
  int bicepVal = analogRead(bicepPot);

  Serial.printf("Lat: %d | Front: %d | Twist: %d | Bicep: %d\n",
                latVal, frontVal, twistVal, bicepVal);

  // Example: test movements
  moveLateralUp();
  moveTwistRight();
  // moveFrontUp();
  // moveBicepUp();

  delay(100);
}
