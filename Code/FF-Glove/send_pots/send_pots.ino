#include <Arduino.h>

//basic ff-glove position tracking


//Pin definitions for the servos
//#define PIN_YAW_SERVO 14
#define POT_THUMB 18
#define POT_INDEX 5
#define POT_MIDDLE 4
#define POT_RING 2
#define POT_PINKY 15



void setup() {
  Serial.begin(115200);
  pinMode(POT_THUMB, INPUT);
  pinMode(POT_INDEX, INPUT);
  pinMode(POT_MIDDLE, INPUT);
  pinMode(POT_RING, INPUT);
  pinMode(POT_PINKY, INPUT);
}

void loop() {
  
  int middle = convert_pot2ang(analogRead(POT_MIDDLE), 0, 180, 2550, 4095);
  int ring = convert_pot2ang(analogRead(POT_RING), 0, 180, 890, 1440);
  int pinky = convert_pot2ang(analogRead(POT_PINKY), 0, 180, 580, 1000);
  
  Serial.print(middle);
  Serial.print(", ");
  Serial.print(ring);
  Serial.print(", ");
  Serial.println(pinky);

  /*
  Serial.print(analogRead(POT_THUMB));
  Serial.print(", ");
  Serial.print(analogRead(POT_INDEX));
  Serial.print(", ");
  Serial.print(analogRead(POT_MIDDLE));
  Serial.print(", ");
  Serial.print(analogRead(POT_RING));
  Serial.print(", ");
  Serial.println(analogRead(POT_PINKY));
  */
}


int convert_pot2ang(int pot_val, int desired_min, int desired_max, int measured_min, int measured_max){
    if (float(pot_val) > measured_max) {
        pot_val = measured_max;
    } else if (pot_val < measured_min) {
        pot_val = measured_min;
    }

    //Calculate the actual and desired ranges
    float actual_range = measured_max - measured_min;
    float desired_range = desired_max - desired_min;

    //Map the actual potentiometer value to the 0-1 range within the actual range
    float normalized_value = float(pot_val - measured_min) / float(actual_range);

    //Convert to the desired range
    int converted_value = (normalized_value * desired_range) + desired_min;

    return converted_value;
}