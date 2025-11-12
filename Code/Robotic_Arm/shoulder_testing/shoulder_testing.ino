#include <Arduino.h>
#include <ESP32Servo.h>

//BICEP TEST


//Pin definitions for the servos
#define PIN_SERVO_1 18
#define PIN_SERVO_2 19
//#define PIN_YAW_SERVO 14
#define PIN_POT 2

bool moveClockwise = true;
bool isFlexing = true;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_SERVO_1, OUTPUT);
  pinMode(PIN_SERVO_2, OUTPUT);
}

void loop() {
  
  int potVal = analogRead(PIN_POT);
  Serial.println(analogRead(PIN_POT));

  
    
  digitalWrite(PIN_SERVO_1, HIGH); 
  digitalWrite(PIN_SERVO_2, LOW);

    // if (potVal <= 10 ){ 
    //   //Stop Motor 
    //   isFlexing = true;
    // }else if(potVal > 1150)
    //   isFlexing = false;

    // if (isFlexing)
    //   flex_in();
    // else
    //   extend_out();
    
}

// void flex_in() {
//   digitalWrite(PIN_SERVO_1, LOW); 
//   digitalWrite(PIN_SERVO_2, HIGH);
//   Serial.println("Flex");
// }

// void extend_out() {
//   digitalWrite(PIN_SERVO_1, HIGH); 
//   digitalWrite(PIN_SERVO_2, LOW);
//   Serial.println("Extend");
// }



