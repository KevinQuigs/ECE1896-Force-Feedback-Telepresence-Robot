#pragma once
#include "../../Config.h"
#include "ESP32Servo.h"


class Haptics {
private:
    Servo pinkyServo;
    Servo ringServo;
    Servo middleServo;
    Servo indexServo;
    Servo thumbServo;

public:
    void setupServoHaptics();
    void scaleLimits(int* hapticLimits, float* scaledLimits);
    void dynScaleLimits(int* hapticLimits, float* scaledLimits);
    void writeServoHaptics(int* hapticLimits);
};