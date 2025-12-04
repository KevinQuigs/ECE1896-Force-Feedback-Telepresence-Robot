#include <Arduino.h>

// ADC pins
#define POT_THUMB 34
#define POT_INDEX 35
#define POT_MIDDLE 13
#define POT_RING 12
#define POT_PINKY 14
#define POT_CALIB 5

void setup() {
  Serial.begin(112500);

  pinMode(POT_THUMB, INPUT);
  pinMode(POT_INDEX, INPUT);
  pinMode(POT_MIDDLE, INPUT);
  pinMode(POT_RING, INPUT);
  pinMode(POT_PINKY, INPUT);
  pinMode(PIN_CALIB, INPUT_PULLUP);
}

void loop() {

  // Convert all 5 fingers
  int thumb = 180 - convert_pot2ang(analogRead(POT_THUMB), 0, 180, 0, 2270);
  int index = 180 - convert_pot2ang(analogRead(POT_INDEX), 0, 180, 0, 1780);
  int middle = 180 - convert_pot2ang(analogRead(POT_MIDDLE), 0, 180, 2300, 4095);
  int ring   = 180 - convert_pot2ang(analogRead(POT_RING),   0, 180, 1320, 3100);
  int pinky  = 180 - convert_pot2ang(analogRead(POT_PINKY),  0, 180, 75, 1800);

  // int thumb = map(analogRead(POT_THUMB), 0, 2270, 0, 180);
  // int index = map(analogRead(POT_INDEX), 0, 1780, 0, 180);
  // int middle = map(analogRead(POT_MIDDLE), 2300, 4095, 0, 180);
  // int ring   = map(analogRead(POT_RING), 1320, 3100, 0, 180);
  // int pinky  = map(analogRead(POT_PINKY), 75, 1800, 0, 180);

  // int thumb = analogRead(POT_THUMB);
  // int index = analogRead(POT_INDEX);
  // int middle = analogRead(POT_MIDDLE);
  // int ring   = analogRead(POT_RING);
  // int pinky  = analogRead(POT_PINKY);
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
  Serial.print(pinky);   // newline at the very end
 
  Serial.print(",");
  Serial.println(digitalRead(POT_CALIB));

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
