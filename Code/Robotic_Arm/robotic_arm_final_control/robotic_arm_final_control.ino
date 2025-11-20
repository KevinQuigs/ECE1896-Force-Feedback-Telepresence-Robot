#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

// ======= CHANGE THIS TO SENDER MAC =======
uint8_t senderMAC[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14};

// ======================= JOINT DEFINITIONS =======================
const int shoulderLatPot = 35;
const int shoulderLatServo1 = 4;
const int shoulderLatServo2 = 16;

const int shoulderFrontPot = 32;
const int shoulderFrontServo1 = 2;
const int shoulderFrontServo2 = 15;

const int shoulderTwistPot = 34;
const int shoulderTwistServo1 = 5;
const int shoulderTwistServo2 = 17;

const int bicepPot = 33;
const int bicepServo1 = 12;
const int bicepServo2 = 13;

// ======================= POT LIMITS =======================
int shoulderLatMin = 1450;
int shoulderLatMax = 1950;

int shoulderFrontMin = 250;
int shoulderFrontMax = 1000;

int shoulderTwistMin = 150;
int shoulderTwistMax = 4000;

int bicepMin = 50;
int bicepMax = 1800;

// ======================= ROBOT DIMENSIONS (meters) =======================
const float EYE_TO_SHOULDER_X = 0.2178;  // right
const float EYE_TO_SHOULDER_Y = -0.2213; // down
const float SHOULDER_TO_ELBOW = 0.2794;
const float ELBOW_TO_HAND = 0.2349;

// ======================= POT/ANGLE CONVERSION CONSTANTS =======================
// Lateral: 1450 pot = 0°, ranges to ~35° at 1950
const float LAT_NEUTRAL_POT = 1450.0;
const float LAT_NEUTRAL_ANGLE = 0.0;
const float LAT_POT_PER_DEGREE = (1950.0 - 1450.0) / 35.0; // ~14.3 pot/degree

// Front: 550 pot = 0°, ranges to ~35° at 1000
const float FRONT_NEUTRAL_POT = 550.0;
const float FRONT_NEUTRAL_ANGLE = 0.0;
const float FRONT_POT_PER_DEGREE = (1000.0 - 550.0) / 35.0; // ~12.86 pot/degree

// Twist: 1000 pot = 0° (straight ahead)
const float TWIST_NEUTRAL_POT = 1000.0;
const float TWIST_POT_PER_DEGREE = 4095.0 / 270.0; // 15.17 pot/degree

// Elbow: 50 pot = 30°, 1800 pot = 90°
const float ELBOW_MIN_ANGLE = 30.0;
const float ELBOW_MIN_POT = 50.0;
const float ELBOW_POT_PER_DEGREE = (1800.0 - 50.0) / (90.0 - 30.0); // 29.17 pot/degree

// ======================= IK TARGET & STATE =======================
struct IKTarget {
  float x, y, z;  // target position relative to eyes
  bool active;
};

IKTarget ikTarget = {0, 0, 0, false};

struct JointAngles {
  float lateral;    // degrees
  float front;      // degrees  
  float twist;      // degrees
  float elbow;      // degrees
};

JointAngles targetAngles = {0, 0, 0, 30};
JointAngles currentAngles = {0, 0, 0, 30};

// ======================= ACTIVE COMMAND =======================
String activeCommand = "";

// ======================= CONVERSION FUNCTIONS =======================
float potToAngleLateral(int pot) {
  return LAT_NEUTRAL_ANGLE + (pot - LAT_NEUTRAL_POT) / LAT_POT_PER_DEGREE;
}

int angleToPotLateral(float angle) {
  return (int)(LAT_NEUTRAL_POT + angle * LAT_POT_PER_DEGREE);
}

float potToAngleFront(int pot) {
  return FRONT_NEUTRAL_ANGLE + (pot - FRONT_NEUTRAL_POT) / FRONT_POT_PER_DEGREE;
}

int angleToPotFront(float angle) {
  return (int)(FRONT_NEUTRAL_POT + angle * FRONT_POT_PER_DEGREE);
}

float potToAngleTwist(int pot) {
  return (pot - TWIST_NEUTRAL_POT) / TWIST_POT_PER_DEGREE;
}

int angleToPotTwist(float angle) {
  return (int)(TWIST_NEUTRAL_POT + angle * TWIST_POT_PER_DEGREE);
}

float potToAngleElbow(int pot) {
  return ELBOW_MIN_ANGLE + (pot - ELBOW_MIN_POT) / ELBOW_POT_PER_DEGREE;
}

int angleToPotElbow(float angle) {
  return (int)(ELBOW_MIN_POT + (angle - ELBOW_MIN_ANGLE) * ELBOW_POT_PER_DEGREE);
}

// ======================= UPDATE CURRENT ANGLES FROM POTS =======================
void updateCurrentAngles() {
  currentAngles.lateral = potToAngleLateral(analogRead(shoulderLatPot));
  currentAngles.front = potToAngleFront(analogRead(shoulderFrontPot));
  currentAngles.twist = potToAngleTwist(analogRead(shoulderTwistPot));
  currentAngles.elbow = potToAngleElbow(analogRead(bicepPot));
}

// ======================= INVERSE KINEMATICS SOLVER =======================
bool solveIK(float target_x, float target_y, float target_z, JointAngles &result) {
  // Convert target from eye frame to shoulder frame
  float shoulder_x = target_x - EYE_TO_SHOULDER_X;
  float shoulder_y = target_y - EYE_TO_SHOULDER_Y;
  float shoulder_z = target_z;

  // Calculate distance from shoulder to target
  float distance = sqrt(shoulder_x*shoulder_x + shoulder_y*shoulder_y + shoulder_z*shoulder_z);
  
  // Check if target is reachable
  float maxReach = SHOULDER_TO_ELBOW + ELBOW_TO_HAND;
  float minReach = fabs(SHOULDER_TO_ELBOW - ELBOW_TO_HAND);
  
  if (distance > maxReach || distance < minReach) {
    Serial.println("Target out of reach!");
    return false;
  }

  // Solve for twist angle (rotation around Y axis)
  result.twist = atan2(shoulder_x, shoulder_z) * 180.0 / PI;
  
  // Project into the plane of the arm (after twist)
  float r_horizontal = sqrt(shoulder_x*shoulder_x + shoulder_z*shoulder_z);
  
  // Solve for elbow angle using law of cosines
  float cos_elbow = (SHOULDER_TO_ELBOW*SHOULDER_TO_ELBOW + ELBOW_TO_HAND*ELBOW_TO_HAND - distance*distance) 
                    / (2.0 * SHOULDER_TO_ELBOW * ELBOW_TO_HAND);
  cos_elbow = constrain(cos_elbow, -1.0, 1.0);
  result.elbow = acos(cos_elbow) * 180.0 / PI;
  
  // Solve for shoulder angles
  // First, find the angle to the target in the arm plane
  float angle_to_target = atan2(shoulder_y, r_horizontal) * 180.0 / PI;
  
  // Then account for the elbow bend
  float cos_shoulder = (SHOULDER_TO_ELBOW*SHOULDER_TO_ELBOW + distance*distance - ELBOW_TO_HAND*ELBOW_TO_HAND)
                       / (2.0 * SHOULDER_TO_ELBOW * distance);
  cos_shoulder = constrain(cos_shoulder, -1.0, 1.0);
  float shoulder_bend = acos(cos_shoulder) * 180.0 / PI;
  
  // Total shoulder angle from vertical
  float total_shoulder = angle_to_target + shoulder_bend;
  
  // Decompose into front and lateral components
  // When arm is raised forward and up, that's primarily "front"
  // When arm is raised to the side, that's "lateral"
  result.front = total_shoulder * (r_horizontal / distance);
  result.lateral = total_shoulder * (fabs(shoulder_x) / distance);
  
  // Apply physical constraints
  result.lateral = constrain(result.lateral, 0, 35);
  result.front = constrain(result.front, 0, 35);
  result.twist = constrain(result.twist, 
    potToAngleTwist(shoulderTwistMin), 
    potToAngleTwist(shoulderTwistMax));
  result.elbow = constrain(result.elbow, 30, 90);

  Serial.printf("IK Solution: Lat=%.1f° Front=%.1f° Twist=%.1f° Elbow=%.1f°\n",
    result.lateral, result.front, result.twist, result.elbow);
  
  return true;
}

// ======================= MOVEMENT CONTROL =======================
const float ANGLE_TOLERANCE = 2.0; // degrees

void moveToTargetAngles() {
  updateCurrentAngles();
  
  // Lateral movement
  float latError = targetAngles.lateral - currentAngles.lateral;
  if (fabs(latError) > ANGLE_TOLERANCE) {
    if (latError > 0) {
      int pot = analogRead(shoulderLatPot);
      if (pot < shoulderLatMax) {
        digitalWrite(shoulderLatServo1, HIGH);
        digitalWrite(shoulderLatServo2, LOW);
      } else {
        digitalWrite(shoulderLatServo1, LOW);
        digitalWrite(shoulderLatServo2, LOW);
      }
    } else {
      int pot = analogRead(shoulderLatPot);
      if (pot > shoulderLatMin) {
        digitalWrite(shoulderLatServo1, LOW);
        digitalWrite(shoulderLatServo2, HIGH);
      } else {
        digitalWrite(shoulderLatServo1, LOW);
        digitalWrite(shoulderLatServo2, LOW);
      }
    }
  } else {
    digitalWrite(shoulderLatServo1, LOW);
    digitalWrite(shoulderLatServo2, LOW);
  }

  // Front movement
  float frontError = targetAngles.front - currentAngles.front;
  if (fabs(frontError) > ANGLE_TOLERANCE) {
    if (frontError > 0) {
      int pot = analogRead(shoulderFrontPot);
      if (pot < shoulderFrontMax) {
        digitalWrite(shoulderFrontServo1, HIGH);
        digitalWrite(shoulderFrontServo2, LOW);
      } else {
        digitalWrite(shoulderFrontServo1, LOW);
        digitalWrite(shoulderFrontServo2, LOW);
      }
    } else {
      int pot = analogRead(shoulderFrontPot);
      if (pot > shoulderFrontMin) {
        digitalWrite(shoulderFrontServo1, LOW);
        digitalWrite(shoulderFrontServo2, HIGH);
      } else {
        digitalWrite(shoulderFrontServo1, LOW);
        digitalWrite(shoulderFrontServo2, LOW);
      }
    }
  } else {
    digitalWrite(shoulderFrontServo1, LOW);
    digitalWrite(shoulderFrontServo2, LOW);
  }

  // Twist movement
  float twistError = targetAngles.twist - currentAngles.twist;
  if (fabs(twistError) > ANGLE_TOLERANCE) {
    if (twistError > 0) {
      int pot = analogRead(shoulderTwistPot);
      if (pot < shoulderTwistMax) {
        digitalWrite(shoulderTwistServo1, HIGH);
        digitalWrite(shoulderTwistServo2, LOW);
      } else {
        digitalWrite(shoulderTwistServo1, LOW);
        digitalWrite(shoulderTwistServo2, LOW);
      }
    } else {
      int pot = analogRead(shoulderTwistPot);
      if (pot > shoulderTwistMin) {
        digitalWrite(shoulderTwistServo1, LOW);
        digitalWrite(shoulderTwistServo2, HIGH);
      } else {
        digitalWrite(shoulderTwistServo1, LOW);
        digitalWrite(shoulderTwistServo2, LOW);
      }
    }
  } else {
    digitalWrite(shoulderTwistServo1, LOW);
    digitalWrite(shoulderTwistServo2, LOW);
  }

  // Elbow movement
  float elbowError = targetAngles.elbow - currentAngles.elbow;
  if (fabs(elbowError) > ANGLE_TOLERANCE) {
    if (elbowError > 0) {
      int pot = analogRead(bicepPot);
      if (pot < bicepMax) {
        digitalWrite(bicepServo1, HIGH);
        digitalWrite(bicepServo2, LOW);
      } else {
        digitalWrite(bicepServo1, LOW);
        digitalWrite(bicepServo2, LOW);
      }
    } else {
      int pot = analogRead(bicepPot);
      if (pot > bicepMin) {
        digitalWrite(bicepServo1, LOW);
        digitalWrite(bicepServo2, HIGH);
      } else {
        digitalWrite(bicepServo1, LOW);
        digitalWrite(bicepServo2, LOW);
      }
    }
  } else {
    digitalWrite(bicepServo1, LOW);
    digitalWrite(bicepServo2, LOW);
  }
}

void stopAllMotors() {
  digitalWrite(shoulderLatServo1, LOW);
  digitalWrite(shoulderLatServo2, LOW);
  digitalWrite(shoulderFrontServo1, LOW);
  digitalWrite(shoulderFrontServo2, LOW);
  digitalWrite(shoulderTwistServo1, LOW);
  digitalWrite(shoulderTwistServo2, LOW);
  digitalWrite(bicepServo1, LOW);
  digitalWrite(bicepServo2, LOW);
}

// ======================= ESP-NOW RECEIVE CALLBACK =======================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len <= 0) return;

  String cmd = String((char*)data);
  cmd.trim();
  Serial.print("CMD received: ");
  Serial.println(cmd);

  // Check for IK target command format: "goto x,y,z"
  if (cmd.startsWith("goto ")) {
    cmd.remove(0, 5); // remove "goto "
    
    // Parse x,y,z
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.lastIndexOf(',');
    
    if (comma1 > 0 && comma2 > comma1) {
      float x = cmd.substring(0, comma1).toFloat();
      float y = cmd.substring(comma1 + 1, comma2).toFloat();
      float z = cmd.substring(comma2 + 1).toFloat();
      
      Serial.printf("Moving to target: X=%.3f Y=%.3f Z=%.3f\n", x, y, z);
      
      if (solveIK(x, y, z, targetAngles)) {
        ikTarget.x = x;
        ikTarget.y = y;
        ikTarget.z = z;
        ikTarget.active = true;
        activeCommand = "ik_move";
      }
    }
  }
  else if (cmd == "stop_all") {
    activeCommand = "";
    ikTarget.active = false;
    stopAllMotors();
  }
  else {
    // Original manual control commands
    activeCommand = cmd;
    ikTarget.active = false;
  }
}

// ======================= SEND POT VALUES BACK =======================
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 500;

void sendPotValues() {
  updateCurrentAngles();
  
  char msg[120];
  snprintf(msg, sizeof(msg), "Lat:%.1f° Front:%.1f° Twist:%.1f° Elbow:%.1f° | Pots: %d,%d,%d,%d",
    currentAngles.lateral, currentAngles.front, currentAngles.twist, currentAngles.elbow,
    analogRead(shoulderLatPot), analogRead(shoulderFrontPot), 
    analogRead(shoulderTwistPot), analogRead(bicepPot));

  esp_err_t res = esp_now_send(senderMAC, (uint8_t*)msg, strlen(msg) + 1);
  if (res != ESP_OK) {
    uint8_t bcast[6];
    memset(bcast, 0xFF, 6);
    esp_now_send(bcast, (uint8_t*)msg, strlen(msg) + 1);
  }
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.print("This device MAC: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while(true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, senderMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Warning: could not add sender as peer.");
  }

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

  stopAllMotors();

  Serial.println("IK Controller ready!");
  Serial.println("Send 'goto x,y,z' to move to position (meters, relative to eyes)");
  Serial.println("Send manual commands: lateral_up, lateral_down, front_up, etc.");
}

// ======================= LOOP =======================
void loop() {
  unsigned long now = millis();

  if (activeCommand == "ik_move" && ikTarget.active) {
    // IK movement mode - move all joints toward target angles
    moveToTargetAngles();
    
    // Check if we've reached the target
    updateCurrentAngles();
    if (fabs(targetAngles.lateral - currentAngles.lateral) < ANGLE_TOLERANCE &&
        fabs(targetAngles.front - currentAngles.front) < ANGLE_TOLERANCE &&
        fabs(targetAngles.twist - currentAngles.twist) < ANGLE_TOLERANCE &&
        fabs(targetAngles.elbow - currentAngles.elbow) < ANGLE_TOLERANCE) {
      Serial.println("Target reached!");
      ikTarget.active = false;
      activeCommand = "";
      stopAllMotors();
    }
  }
  else if (activeCommand != "" && activeCommand != "ik_move") {
    // Original manual control mode
    int latPot = analogRead(shoulderLatPot);
    if (activeCommand == "lateral_up" && latPot < shoulderLatMax)
      digitalWrite(shoulderLatServo1, HIGH), digitalWrite(shoulderLatServo2, LOW);
    else if (activeCommand == "lateral_down" && latPot > shoulderLatMin)
      digitalWrite(shoulderLatServo1, LOW), digitalWrite(shoulderLatServo2, HIGH);
    else
      digitalWrite(shoulderLatServo1, LOW), digitalWrite(shoulderLatServo2, LOW);

    int frontPot = analogRead(shoulderFrontPot);
    if (activeCommand == "front_up" && frontPot < shoulderFrontMax)
      digitalWrite(shoulderFrontServo1, HIGH), digitalWrite(shoulderFrontServo2, LOW);
    else if (activeCommand == "front_down" && frontPot > shoulderFrontMin)
      digitalWrite(shoulderFrontServo1, LOW), digitalWrite(shoulderFrontServo2, HIGH);
    else
      digitalWrite(shoulderFrontServo1, LOW), digitalWrite(shoulderFrontServo2, LOW);

    int twistPot = analogRead(shoulderTwistPot);
    if (activeCommand == "twist_right" && twistPot < shoulderTwistMax)
      digitalWrite(shoulderTwistServo1, HIGH), digitalWrite(shoulderTwistServo2, LOW);
    else if (activeCommand == "twist_left" && twistPot > shoulderTwistMin)
      digitalWrite(shoulderTwistServo1, LOW), digitalWrite(shoulderTwistServo2, HIGH);
    else
      digitalWrite(shoulderTwistServo1, LOW), digitalWrite(shoulderTwistServo2, LOW);

    int bicepPotVal = analogRead(bicepPot);
    if (activeCommand == "elbow_up" && bicepPotVal < bicepMax)
      digitalWrite(bicepServo1, HIGH), digitalWrite(bicepServo2, LOW);
    else if (activeCommand == "elbow_down" && bicepPotVal > bicepMin)
      digitalWrite(bicepServo1, LOW), digitalWrite(bicepServo2, HIGH);
    else
      digitalWrite(bicepServo1, LOW), digitalWrite(bicepServo2, LOW);
  }

  // Periodic feedback
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    sendPotValues();
  }
  
  delay(1);
}