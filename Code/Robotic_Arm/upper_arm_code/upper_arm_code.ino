#include <Arduino.h>
#include <math.h>

// ---------- Simple 3D vector (put this at the very top) ----------
struct Vec3 {
  float x;
  float y;
  float z;
};

Vec3 v(float x, float y, float z) {
  Vec3 out; 
  out.x = x; 
  out.y = y; 
  out.z = z; 
  return out;
}
Vec3 add(const Vec3 &a, const Vec3 &b) {
  Vec3 out; out.x = a.x + b.x; out.y = a.y + b.y; out.z = a.z + b.z; return out;
}
Vec3 subV(const Vec3 &a, const Vec3 &b) {
  Vec3 out; out.x = a.x - b.x; out.y = a.y - b.y; out.z = a.z - b.z; return out;
}
Vec3 mul(const Vec3 &a, float s) {
  Vec3 out; out.x = a.x * s; out.y = a.y * s; out.z = a.z * s; return out;
}
float dot(const Vec3 &a, const Vec3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
float len(const Vec3 &a) { return sqrtf(dot(a,a)); }
Vec3 normalize(const Vec3 &a) {
  float L = len(a);
  if (L < 1e-6f) return v(0,0,0);
  return mul(a, 1.0f / L);
}




// ---------- Pins (your values) ----------
const int shoulderLatPot = 25;
const int shoulderLatServo1 = 26; // H-bridge IN1
const int shoulderLatServo2 = 27; // H-bridge IN2

const int shoulderFrontPot = 14;
const int shoulderFrontServo1 = 12;
const int shoulderFrontServo2 = 13;

const int shoulderTwistPot = 4;
const int shoulderTwistServo1 = 18;
const int shoulderTwistServo2 = 19;

const int bicepPot = 2;
const int bicepServo1 = 16;
const int bicepServo2 = 17;

// ---------- Pot limits (use the numbers you gave) ----------
int shoulderLatMin = 2050;
int shoulderLatMax = 2500;

int shoulderFrontMin = 880;    // arm at side ~ 0°
int shoulderFrontMax = 2300;   // arm straight forward ~ 90°

int shoulderTwistMin = 500;
int shoulderTwistMax = 3500;

int bicepMin = 10;    // pot near 10 => 0°
int bicepMax = 1500;  // approx => ~100°

// ---------- Physical dims (mm) ----------
const float shoulder_x = 217.805f;
const float shoulder_y = -212.3186f;
const float shoulder_z = 0.0f;

const float upper_arm_len = 279.4f;   // mm
const float forearm_len   = 234.95f;  // mm

// ---------- Angle limits (degrees) ----------
const float frontMinAngle = 0.0f;     // pot=880
const float frontMaxAngle = 90.0f;    // pot=2300

const float latMinAngle = 0.0f;       // pot=2050 (arm down)
const float latMaxAngle = 90.0f;      // pot=2500 (arm up to side)

const float twistMinAngle = -180.0f;  // choose wide range
const float twistMaxAngle = 180.0f;

const float elbowMinAngle = 0.0f;     // extended
const float elbowMaxAngle = 100.0f;   // flexed

// ---------- CCD settings ----------
const float CCD_STEP_DEG = 2.0f;        // change per iteration when adjusting a DOF
const float TARGET_THRESHOLD = 10.0f;   // mm; stop when end effector within this distance
const int   CCD_MAX_ITERS = 200;

// ---------- Utility ----------
float deg2rad(float d) { return d * 3.14159265358979323846f / 180.0f; }
float rad2deg(float r) { return r * 180.0f / 3.14159265358979323846f; }


// ---------- Pot <-> Angle mappings (linear) ----------
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return out_min + t * (out_max - out_min);
}

float potToFrontAngle(int pot) {
  return mapFloat((float)pot, (float)shoulderFrontMin, (float)shoulderFrontMax, frontMinAngle, frontMaxAngle);
}
float frontAngleToPot(float angle) {
  return mapFloat(angle, frontMinAngle, frontMaxAngle, (float)shoulderFrontMin, (float)shoulderFrontMax);
}

float potToLatAngle(int pot) {
  return mapFloat((float)pot, (float)shoulderLatMin, (float)shoulderLatMax, latMinAngle, latMaxAngle);
}
float latAngleToPot(float angle) {
  return mapFloat(angle, latMinAngle, latMaxAngle, (float)shoulderLatMin, (float)shoulderLatMax);
}

float potToTwistAngle(int pot) {
  return mapFloat((float)pot, (float)shoulderTwistMin, (float)shoulderTwistMax, twistMinAngle, twistMaxAngle);
}
float twistAngleToPot(float angle) {
  return mapFloat(angle, twistMinAngle, twistMaxAngle, (float)shoulderTwistMin, (float)shoulderTwistMax);
}

float potToElbowAngle(int pot) {
  return mapFloat((float)pot, (float)bicepMin, (float)bicepMax, elbowMinAngle, elbowMaxAngle);
}
float elbowAngleToPot(float angle) {
  return mapFloat(angle, elbowMinAngle, elbowMaxAngle, (float)bicepMin, (float)bicepMax);
}

// ---------- Movement helpers (assume these exist directly) ----------
void moveLateralUp();
void moveLateralDown();
void stopLateral();

void moveFrontUp();
void moveFrontDown();
void stopFront();

void moveTwistRight();
void moveTwistLeft();
void stopTwist();

void moveBicepUp();
void moveBicepDown();
void stopBicep();

// ---------- Read current angles from pots ----------
void readJointAngles(float &twistDeg, float &frontDeg, float &latDeg, float &elbowDeg) {
  int pTwist = analogRead(shoulderTwistPot);
  int pFront = analogRead(shoulderFrontPot);
  int pLat   = analogRead(shoulderLatPot);
  int pElbow = analogRead(bicepPot);

  twistDeg = potToTwistAngle(pTwist);
  frontDeg = potToFrontAngle(pFront);
  latDeg   = potToLatAngle(pLat);
  elbowDeg = potToElbowAngle(pElbow);
}

// ---------- Forward kinematics: compute end effector (wrist) position ----------
Vec3 forwardKinematics(float twistDeg, float latDeg, float frontDeg, float elbowDeg) {
  // Convert to radians
  float t = deg2rad(twistDeg);
  float a = deg2rad(latDeg);
  float p = deg2rad(frontDeg);
  float e = deg2rad(elbowDeg);

  // Start at shoulder world position
  Vec3 pos = v(shoulder_x, shoulder_y, shoulder_z);

  // initial upper-arm direction: forward along +X
  Vec3 dir = v(1.0f, 0.0f, 0.0f);

  // apply yaw (twist) around Y
  Vec3 dir1;
  dir1.x = dir.x * cosf(t) + dir.z * sinf(t);
  dir1.y = dir.y;
  dir1.z = -dir.x * sinf(t) + dir.z * cosf(t);

  // apply roll (lateral) around local Z
  float ca = cosf(a), sa = sinf(a);
  Vec3 dir2;
  dir2.x = dir1.x * ca - dir1.y * sa;
  dir2.y = dir1.x * sa + dir1.y * ca;
  dir2.z = dir1.z;

  // apply pitch (front) around local X
  float cp = cosf(p), sp = sinf(p);
  Vec3 dir3;
  dir3.x = dir2.x;
  dir3.y = dir2.y * cp - dir2.z * sp;
  dir3.z = dir2.y * sp + dir2.z * cp;

  // upper arm end (elbow) position
  Vec3 elbowPos = add(pos, mul(normalize(dir3), upper_arm_len));

  // Approximate forearm direction by creating orthonormal basis and folding by elbow angle
  Vec3 x = normalize(dir3);
  Vec3 up = v(0.0f, 1.0f, 0.0f);

  // z = normalized(up - proj_up_on_x)
  Vec3 proj = mul(x, dot(up, x));
  Vec3 zVec = subV(up, proj);
  float zLen = len(zVec);
  if (zLen < 1e-6f) {
    zVec = v(0.0f, 0.0f, 1.0f);
  } else {
    zVec = mul(zVec, 1.0f / zLen);
  }

  // y = cross(z, x)  (cross product)
  Vec3 yVec;
  yVec.x = zVec.y * x.z - zVec.z * x.y;
  yVec.y = zVec.z * x.x - zVec.x * x.z;
  yVec.z = zVec.x * x.y - zVec.y * x.x;
  yVec = normalize(yVec);

  // rotate x by -e around yVec (approx hinge)
  float ce = cosf(-e), se = sinf(-e);
  Vec3 dir4 = add(mul(x, ce), mul(zVec, se));

  Vec3 wristPos = add(elbowPos, mul(normalize(dir4), forearm_len));
  return wristPos;
}

// ---------- Motor command: move toward desired angle ----------
void commandJointToAngle(float currentDeg, float targetDeg,
                         void (*movePos)(), void (*moveNeg)(), void (*stop)()) {
  float err = targetDeg - currentDeg;
  if (fabs(err) < 1.0f) {
    stop();
    return;
  }
  if (err > 0.0f) movePos();
  else moveNeg();
}

// ---------- CCD algorithm (iterative) ----------
// joints: order of adjustment from outermost to innermost for CCD: elbow -> lateral -> front -> twist
bool runCCD(Vec3 target, float &twistDeg, float &frontDeg, float &latDeg, float &elbowDeg) {
  for (int iter = 0; iter < CCD_MAX_ITERS; ++iter) {
    Vec3 wrist = forwardKinematics(twistDeg, latDeg, frontDeg, elbowDeg);
    float dist = len(subV(target, wrist));
    if (dist <= TARGET_THRESHOLD) return true;

    // 1) Elbow
    {
      float bestAngle = elbowDeg;
      float bestDist = dist;
      for (int sign = -1; sign <= 1; sign += 2) {
        float cand = elbowDeg + sign * CCD_STEP_DEG;
        cand = constrain(cand, elbowMinAngle, elbowMaxAngle);
        Vec3 w = forwardKinematics(twistDeg, latDeg, frontDeg, cand);
        float d = len(subV(target, w));
        if (d < bestDist) { bestDist = d; bestAngle = cand; }
      }
      elbowDeg = bestAngle;
    }

    // 2) Lateral
    {
      float bestAngle = latDeg;
      float bestDist = len(subV(target, forwardKinematics(twistDeg, latDeg, frontDeg, elbowDeg)));
      for (int sign = -1; sign <= 1; sign += 2) {
        float cand = latDeg + sign * CCD_STEP_DEG;
        cand = constrain(cand, latMinAngle, latMaxAngle);
        Vec3 w = forwardKinematics(twistDeg, cand, frontDeg, elbowDeg);
        float d = len(subV(target, w));
        if (d < bestDist) { bestDist = d; bestAngle = cand; }
      }
      latDeg = bestAngle;
    }

    // 3) Front
    {
      float bestAngle = frontDeg;
      float bestDist = len(subV(target, forwardKinematics(twistDeg, latDeg, frontDeg, elbowDeg)));
      for (int sign = -1; sign <= 1; sign += 2) {
        float cand = frontDeg + sign * CCD_STEP_DEG;
        cand = constrain(cand, frontMinAngle, frontMaxAngle);
        Vec3 w = forwardKinematics(twistDeg, latDeg, cand, elbowDeg);
        float d = len(subV(target, w));
        if (d < bestDist) { bestDist = d; bestAngle = cand; }
      }
      frontDeg = bestAngle;
    }

    // 4) Twist
    {
      float bestAngle = twistDeg;
      float bestDist = len(subV(target, forwardKinematics(twistDeg, latDeg, frontDeg, elbowDeg)));
      for (int sign = -1; sign <= 1; sign += 2) {
        float cand = twistDeg + sign * CCD_STEP_DEG;
        if (cand < twistMinAngle) cand = twistMinAngle;
        if (cand > twistMaxAngle) cand = twistMaxAngle;
        Vec3 w = forwardKinematics(cand, latDeg, frontDeg, elbowDeg);
        float d = len(subV(target, w));
        if (d < bestDist) { bestDist = d; bestAngle = cand; }
      }
      twistDeg = bestAngle;
    }
  }
  return false;
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);

  // pots
  pinMode(shoulderLatPot, INPUT);
  pinMode(shoulderFrontPot, INPUT);
  pinMode(shoulderTwistPot, INPUT);
  pinMode(bicepPot, INPUT);

  // h-bridge pins
  pinMode(shoulderLatServo1, OUTPUT);
  pinMode(shoulderLatServo2, OUTPUT);
  pinMode(shoulderFrontServo1, OUTPUT);
  pinMode(shoulderFrontServo2, OUTPUT);
  pinMode(shoulderTwistServo1, OUTPUT);
  pinMode(shoulderTwistServo2, OUTPUT);
  pinMode(bicepServo1, OUTPUT);
  pinMode(bicepServo2, OUTPUT);

  // init stop all
  stopLateral(); stopFront(); stopTwist(); stopBicep();

  Serial.println("IK/CCD system ready.");
}

void loop() {
  // read current angles from pots
  float twistDeg, frontDeg, latDeg, elbowDeg;
  readJointAngles(twistDeg, frontDeg, latDeg, elbowDeg);

  // target in head-based coords - set these before calling loop or change here
  Vec3 target = v(300.0f, -100.0f, 150.0f); // replace with your desired target (mm), origin=head

  // run CCD to compute desired joint angles (updates twistDeg, frontDeg, latDeg, elbowDeg)
  bool ok = runCCD(target, twistDeg, frontDeg, latDeg, elbowDeg);

  // re-read current to be conservative
  float curTwist, curFront, curLat, curElbow;
  readJointAngles(curTwist, curFront, curLat, curElbow);

  // Command joints (these functions will self-limit using pot checks)
  commandJointToAngle(curTwist, twistDeg, moveTwistRight, moveTwistLeft, stopTwist);
  commandJointToAngle(curFront, frontDeg, moveFrontUp, moveFrontDown, stopFront);
  commandJointToAngle(curLat, latDeg, moveLateralUp, moveLateralDown, stopLateral);
  commandJointToAngle(curElbow, elbowDeg, moveBicepUp, moveBicepDown, stopBicep);

  // debug
  Vec3 wristNow = forwardKinematics(curTwist, curLat, curFront, curElbow);
  Serial.printf("Target (%.1f,%.1f,%.1f) Wrist(%.1f,%.1f,%.1f) CCDok=%d\n",
                target.x,target.y,target.z, wristNow.x,wristNow.y,wristNow.z, ok ? 1:0);

  delay(80);
}

// ---------- Stubs for move/stop functions (paste or keep your safe implementations) ----------
void moveLateralUp()  { int potVal=analogRead(shoulderLatPot); if (potVal < shoulderLatMax) { digitalWrite(shoulderLatServo1,HIGH); digitalWrite(shoulderLatServo2,LOW);} else stopLateral();}
void moveLateralDown(){ int potVal=analogRead(shoulderLatPot); if (potVal > shoulderLatMin) { digitalWrite(shoulderLatServo1,LOW); digitalWrite(shoulderLatServo2,HIGH);} else stopLateral();}
void stopLateral(){ digitalWrite(shoulderLatServo1,LOW); digitalWrite(shoulderLatServo2,LOW);}

void moveFrontUp()   { int pot=analogRead(shoulderFrontPot); if (pot < shoulderFrontMax) { digitalWrite(shoulderFrontServo1,HIGH); digitalWrite(shoulderFrontServo2,LOW);} else stopFront();}
void moveFrontDown() { int pot=analogRead(shoulderFrontPot); if (pot > shoulderFrontMin) { digitalWrite(shoulderFrontServo1,LOW); digitalWrite(shoulderFrontServo2,HIGH);} else stopFront();}
void stopFront(){ digitalWrite(shoulderFrontServo1,LOW); digitalWrite(shoulderFrontServo2,LOW);}

void moveTwistRight(){ int pot=analogRead(shoulderTwistPot); if (pot < shoulderTwistMax) { digitalWrite(shoulderTwistServo1,HIGH); digitalWrite(shoulderTwistServo2,LOW);} else stopTwist();}
void moveTwistLeft() { int pot=analogRead(shoulderTwistPot); if (pot > shoulderTwistMin) { digitalWrite(shoulderTwistServo1,LOW); digitalWrite(shoulderTwistServo2,HIGH);} else stopTwist();}
void stopTwist(){ digitalWrite(shoulderTwistServo1,LOW); digitalWrite(shoulderTwistServo2,LOW);}

void moveBicepUp()   { int pot=analogRead(bicepPot); if (pot < bicepMax) { digitalWrite(bicepServo1,HIGH); digitalWrite(bicepServo2,LOW);} else stopBicep();}
void moveBicepDown() { int pot=analogRead(bicepPot); if (pot > bicepMin) { digitalWrite(bicepServo1,LOW); digitalWrite(bicepServo2,HIGH);} else stopBicep();}
void stopBicep(){ digitalWrite(bicepServo1,LOW); digitalWrite(bicepServo2,LOW);}

