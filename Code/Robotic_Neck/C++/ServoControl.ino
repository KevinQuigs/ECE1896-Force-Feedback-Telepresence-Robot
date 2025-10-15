#include <Servo.h>


Servo yaw; //yaw - servo in the back of the neck. Controls the spinning motion of the entire neck movement.
Servo left; //left - servo on the front-left side of the neck (Robot's POV) 
Servo right; //right - servo on the front-right side of the neck (Robot's POV)
//Servo left and right are used in unison to control pitch and roll of the neck.

const int MAX_ANGLE_YAW = 60; // Maximum angle for yaw servo
const int MIN_ANGLE_YAW = -60; // Minimum angle for yaw servo

void setup() {
  yaw.attach(9); // Attach the servo on pin 9 to the servo object
}

void loop() {
  // Sweep from 0 to 120 degrees
  for (int pos = 0; pos <= 120; pos += 1) {
    myServo.write(pos); // Tell servo to go to position in variable 'pos'
    delay(15);          // Wait for 15 milliseconds for the servo to reach the position
  }

  // Sweep from 120 to 0 degrees
  for (int pos = 120; pos >= 0; pos -= 1) {
    myServo.write(pos); // Tell servo to go to position in variable 'pos'
    delay(15);          // Wait for 15 milliseconds for the servo to reach the position
  }
}







#include <ESP32Servo.h>   // if you use another servo lib, adapt names

Servo servoLeft;
Servo servoRight;

// --- Geometry / calibration parameters (tune these) ---
const float a_mm      = 52.451f;   // forward distance from pivot to attach point (mm)
const float b_mm      = 43.4594f;    // half-track (mm)
const float r_mm      = 141.5034f;    // servo arm length (mm)
const float z_mount_mm = 21.0f;   // mount offset (mm)

// Servo pins
const int PIN_SERVO_LEFT  = 13;   // change to your GPIO
const int PIN_SERVO_RIGHT = 12;   // change to your GPIO

// Servo neutral offsets: adjust if your servos have zero at a different angle
const float servoLeftOffsetDeg  = 0.0f;
const float servoRightOffsetDeg = 0.0f;

// Servo safe limits (degrees)
const float SERVO_MIN = 0.0f;
const float SERVO_MAX = 180.0f;

// Utility
float degToRad(float d) { return d * (PI / 180.0f); }
float radToDeg(float r) { return r * (180.0f / PI); }

// Compute z (mm) at an attachment point given pitch (rad) and roll (rad)
void computeAttachmentHeights(float pitch_rad, float roll_rad, float &z_left, float &z_right) {
  // x = a, y = +/- b
  float a = a_mm;
  float b = b_mm;
  float sin_pitch = sinf(pitch_rad);
  float sin_roll  = sinf(roll_rad);
  float cos_pitch = cosf(pitch_rad);

  // from derived formula: z = -x*sin(pitch) + y * sin(roll) * cos(pitch)
  z_right = - a * sin_pitch + b * sin_roll * cos_pitch;
  z_left  = - a * sin_pitch - b * sin_roll * cos_pitch;
}

// Convert desired z height to servo angle (deg) using simple arc model: z = z_mount + r * sin(alpha)
bool zToServoAngleDeg(float z_mm, float &angle_deg) {
  float arg = (z_mm - z_mount_mm) / r_mm;
  // clamp
  if (arg > 1.0f) arg = 1.0f;
  if (arg < -1.0f) arg = -1.0f;
  float alpha_rad = asinf(arg);  // returns radians
  angle_deg = radToDeg(alpha_rad);
  return true;
}

// Parse a string "(pitch, roll)" where pitch and roll are degrees.
// Example inputs:
// "(10, -5)" or " ( 10  , -5 )\n"
bool parsePitchRoll(const String &s, float &pitchDeg, float &rollDeg) {
  // find two numbers inside parentheses
  int i0 = s.indexOf('(');
  int i1 = s.indexOf(')');
  if (i0 < 0 || i1 < 0 || i1 <= i0) return false;
  String inside = s.substring(i0 + 1, i1);
  inside.trim();
  // replace commas with space to simplify
  inside.replace(',', ' ');
  // now use sscanf-like parse:
  float p = 0.0f, r = 0.0f;
  // Arduino String -> c_str
  int read = sscanf(inside.c_str(), "%f %f", &p, &r);
  if (read == 2) {
    pitchDeg = p;
    rollDeg  = r;
    return true;
  }
  return false;
}

void applyPitchRoll(float pitchDeg, float rollDeg) {
  // convert to radians
  float pitchRad = degToRad(pitchDeg);
  float rollRad  = degToRad(rollDeg);
  float zL, zR;
  computeAttachmentHeights(pitchRad, rollRad, zL, zR);

  float angleLeftDeg, angleRightDeg;
  zToServoAngleDeg(zL, angleLeftDeg);
  zToServoAngleDeg(zR, angleRightDeg);

  // Apply offsets (calibration)
  angleLeftDeg  += servoLeftOffsetDeg;
  angleRightDeg += servoRightOffsetDeg;

  // clamp to servo safe range
  if (angleLeftDeg < SERVO_MIN) angleLeftDeg = SERVO_MIN;
  if (angleLeftDeg > SERVO_MAX) angleLeftDeg = SERVO_MAX;
  if (angleRightDeg < SERVO_MIN) angleRightDeg = SERVO_MIN;
  if (angleRightDeg > SERVO_MAX) angleRightDeg = SERVO_MAX;

  // Write to servos
  servoLeft.write((int)round(angleLeftDeg));
  servoRight.write((int)round(angleRightDeg));

  // Debug
  Serial.printf("Set P=%.2f°, R=%.2f° -> zL=%.2f mm, zR=%.2f mm, aL=%.1f°, aR=%.1f°\n",
                pitchDeg, rollDeg, zL, zR, angleLeftDeg, angleRightDeg);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("3-leg pitch-roll controller starting...");

  // attach servos
  servoLeft.attach(PIN_SERVO_LEFT);
  servoRight.attach(PIN_SERVO_RIGHT);

  // Set to neutral
  applyPitchRoll(0.0f, 0.0f);
}

String inputBuffer;

void loop() {
  // Read serial input line(s)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (inputBuffer.length() > 0) {
        float p, r;
        if (parsePitchRoll(inputBuffer, p, r)) {
          applyPitchRoll(p, r);
        } else {
          Serial.printf("Parse error: '%s'\n", inputBuffer.c_str());
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // optional: small delay
  delay(5);
}