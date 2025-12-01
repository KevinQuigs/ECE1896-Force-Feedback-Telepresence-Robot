#include <Arduino.h>

// ADC pins
#define POT_THUMB 34
#define POT_INDEX 35
#define POT_MIDDLE 32
#define POT_RING 33
#define POT_PINKY 36

void setup() {
  Serial.begin(112500);

  pinMode(POT_THUMB, INPUT);
  pinMode(POT_INDEX, INPUT);
  pinMode(POT_MIDDLE, INPUT);
  pinMode(POT_RING, INPUT);
  pinMode(POT_PINKY, INPUT);
}

void loop() {

  // Convert all 5 fingers
  int thumb = convert_pot2ang(analogRead(POT_THUMB), 0, 180, 500, 3500);
  int index = convert_pot2ang(analogRead(POT_INDEX), 0, 180, 500, 3500);
  int middle = convert_pot2ang(analogRead(POT_MIDDLE), 0, 180, 2550, 4095);
  int ring   = convert_pot2ang(analogRead(POT_RING),   0, 180, 890, 1440);
  int pinky  = convert_pot2ang(analogRead(POT_PINKY),  0, 180, 580, 1000);

  // Format: FTxxFIxxFMxxFRxxFPxx
  // Serial.print("FT");
  Serial.print(thumb);

  Serial.print(",");
  Serial.print(index);

  Serial.print(",");
  Serial.print(middle);

  Serial.print(",");
  Serial.print(ring);

  Serial.print(",");
  Serial.println(pinky);   // newline at the very end
  Serial.print(",");

  delay(20);
}

// Map raw pot to angle
int convert_pot2ang(int pot_val, int desired_min, int desired_max, int measured_min, int measured_max){
    if (pot_val > measured_max) pot_val = measured_max;
    else if (pot_val < measured_min) pot_val = measured_min;

    float actual_range = measured_max - measured_min;
    float desired_range = desired_max - desired_min;

    float normalized_value = float(pot_val - measured_min) / actual_range;

    return int(normalized_value * desired_range + desired_min);
}
