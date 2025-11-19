#include <ESP32Servo.h>

const int wristServoPin = 5; // Pin connected to wrist servo
Servo wristServo;

void setup() {
  Serial.begin(115200);  // Start serial communication
  wristServo.attach(wristServoPin); 
  Serial.println("Ready to receive wrist angle (20-160).");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt(); // Read integer from serial
    if (angle >= 20 && angle <= 160) {
      wristServo.write(angle);     // Move servo
      Serial.print("Servo moved to: ");
      Serial.println(angle);
    } else {
      Serial.println("Angle out of range (20-160).");
    }
    Serial.read(); // Clear any leftover newline
  }
}
