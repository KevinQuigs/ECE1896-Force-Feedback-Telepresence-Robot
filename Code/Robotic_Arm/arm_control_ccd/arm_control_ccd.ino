#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

// ======= CHANGE THIS TO SENDER MAC =======
uint8_t senderMAC[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14};

struct Vector3 {
  float x, y, z;
};

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
int shoulderLatMin = 1600;
int shoulderLatMax = 2050;

int shoulderFrontMin = 250;
int shoulderFrontMax = 1000;

int shoulderTwistMin = 150;
int shoulderTwistMax = 4000;

int bicepMin = 200;
int bicepMax = 1800;

// ======================= ROBOT DIMENSIONS (meters) =======================
const float EYE_TO_SHOULDER_X = 0.2178;
const float EYE_TO_SHOULDER_Y = -0.2213;
const float SHOULDER_TO_ELBOW = 0.2794;
const float ELBOW_TO_HAND = 0.2349;

// ======================= POT/ANGLE CONVERSION CONSTANTS =======================
const float LAT_NEUTRAL_POT = 1560.0;
const float LAT_NEUTRAL_ANGLE = 0.0;
const float LAT_POT_PER_DEGREE = (1950.0 - 1560.0) / 35.0;

const float FRONT_NEUTRAL_POT = 550.0;
const float FRONT_NEUTRAL_ANGLE = 0.0;
const float FRONT_POT_PER_DEGREE = (1000.0 - 550.0) / 35.0;

// const float TWIST_NEUTRAL_POT = 1100.0;
const float TWIST_NEUTRAL_POT = 920.0;
const float TWIST_POT_PER_DEGREE = (2080.0 - 920.0) / 45.0;

const float ELBOW_MIN_ANGLE = 35.0;
const float ELBOW_MIN_POT = 50.0;
const float ELBOW_POT_PER_DEGREE = (1300.0 - 50.0) / (90.0 - 30.0);

// ======================= IK TARGET & STATE =======================
struct IKTarget {
  float x, y, z;
  bool active;
};

IKTarget ikTarget = {0, 0, 0, false};

struct JointAngles {
  float lateral;
  float front;
  float twist;
  float elbow;
};

JointAngles targetAngles = {0, 0, 0, 30};
JointAngles currentAngles = {0, 0, 0, 30};

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

void updateCurrentAngles() {
  currentAngles.lateral = potToAngleLateral(analogRead(shoulderLatPot));
  currentAngles.front = potToAngleFront(analogRead(shoulderFrontPot));
  currentAngles.twist = potToAngleTwist(analogRead(shoulderTwistPot));
  currentAngles.elbow = potToAngleElbow(analogRead(bicepPot));
}

// ======================= FORWARD KINEMATICS =======================
// Calculate end effector position from joint angles

Vector3 forwardKinematics(JointAngles angles) {
  // Convert angles to radians
  float twist_rad = angles.twist * PI / 180.0;
  float lat_rad = angles.lateral * PI / 180.0;
  float front_rad = angles.front * PI / 180.0;
  float elbow_rad = angles.elbow * PI / 180.0;
  
  // Start at shoulder (relative to eyes)
  Vector3 pos;
  pos.x = EYE_TO_SHOULDER_X;
  pos.y = EYE_TO_SHOULDER_Y;
  pos.z = 0;
  
  // Apply twist rotation (around Y axis)
  float cos_twist = cos(twist_rad);
  float sin_twist = sin(twist_rad);
  
  // Apply lateral raise (rotation to side)
  float cos_lat = cos(lat_rad);
  float sin_lat = sin(lat_rad);
  
  // Apply front raise (rotation forward)
  float cos_front = cos(front_rad);
  float sin_front = sin(front_rad);
  
  // Upper arm direction (shoulder to elbow)
  // Combine front and lateral rotations, then apply twist
  float upper_y = -cos_lat * cos_front;  // negative because down is negative Y
  float upper_horiz = sin_front;
  
  // After twist rotation
  float upper_x = upper_horiz * sin_twist + sin_lat * cos_twist;
  float upper_z = upper_horiz * cos_twist - sin_lat * sin_twist;
  
  // Normalize and scale by upper arm length
  float upper_mag = sqrt(upper_x*upper_x + upper_y*upper_y + upper_z*upper_z);
  upper_x = (upper_x / upper_mag) * SHOULDER_TO_ELBOW;
  upper_y = (upper_y / upper_mag) * SHOULDER_TO_ELBOW;
  upper_z = (upper_z / upper_mag) * SHOULDER_TO_ELBOW;
  
  // Elbow position
  float elbow_x = pos.x + upper_x;
  float elbow_y = pos.y + upper_y;
  float elbow_z = pos.z + upper_z;
  
  // Forearm direction (continues in same plane, bent by elbow angle)
  // The elbow bends in the plane of the upper arm
  float forearm_extension = sin(elbow_rad);
  float forearm_x = upper_x * (1.0 + forearm_extension / SHOULDER_TO_ELBOW);
  float forearm_y = upper_y * (1.0 + forearm_extension / SHOULDER_TO_ELBOW);
  float forearm_z = upper_z * (1.0 + forearm_extension / SHOULDER_TO_ELBOW);
  
  // Normalize and scale by forearm length
  float forearm_mag = sqrt(forearm_x*forearm_x + forearm_y*forearm_y + forearm_z*forearm_z);
  forearm_x = (forearm_x / forearm_mag) * ELBOW_TO_HAND;
  forearm_y = (forearm_y / forearm_mag) * ELBOW_TO_HAND;
  forearm_z = (forearm_z / forearm_mag) * ELBOW_TO_HAND;
  
  // Final hand position
  pos.x = elbow_x + forearm_x;
  pos.y = elbow_y + forearm_y;
  pos.z = elbow_z + forearm_z;
  
  return pos;
}

// ======================= CCD INVERSE KINEMATICS =======================
const int CCD_MAX_ITERATIONS = 20;
const float CCD_TOLERANCE = 0.01; // 1cm tolerance

bool solveCCD(float target_x, float target_y, float target_z, JointAngles &result) {
  // Start with current angles or neutral position
  result = currentAngles;
  
  // Check basic reachability
  float shoulder_x = target_x - EYE_TO_SHOULDER_X;
  float shoulder_y = target_y - EYE_TO_SHOULDER_Y;
  float shoulder_z = target_z;
  float distance = sqrt(shoulder_x*shoulder_x + shoulder_y*shoulder_y + shoulder_z*shoulder_z);
  
  float maxReach = SHOULDER_TO_ELBOW + ELBOW_TO_HAND;
  float minReach = fabs(SHOULDER_TO_ELBOW - ELBOW_TO_HAND);
  
  bool outOfReach = false;
  if (distance > maxReach * 1.05) {
    Serial.printf("WARNING: Target %.3fm away, max reach is %.3fm - will get as close as possible\n", 
      distance, maxReach);
    outOfReach = true;
  } else if (distance < minReach * 0.95) {
    Serial.printf("WARNING: Target %.3fm away, min reach is %.3fm - will get as close as possible\n", 
      distance, minReach);
    outOfReach = true;
  }
  
  Serial.println("=== Starting CCD Iterations ===");
  
  // CCD iteration loop
  for (int iter = 0; iter < CCD_MAX_ITERATIONS; iter++) {
    // Calculate current end effector position
    Vector3 current_pos = forwardKinematics(result);
    
    // Calculate error
    float error_x = target_x - current_pos.x;
    float error_y = target_y - current_pos.y;
    float error_z = target_z - current_pos.z;
    float error = sqrt(error_x*error_x + error_y*error_y + error_z*error_z);
    
    Serial.printf("Iter %d: Error=%.4fm Current=(%.3f,%.3f,%.3f) Target=(%.3f,%.3f,%.3f)\n",
      iter, error, current_pos.x, current_pos.y, current_pos.z, target_x, target_y, target_z);
    
    // Check convergence
    if (error < CCD_TOLERANCE) {
      Serial.println("=== CCD Converged! ===");
      Serial.printf("Final angles: Lat=%.1f° Front=%.1f° Twist=%.1f° Elbow=%.1f°\n",
        result.lateral, result.front, result.twist, result.elbow);
      return true;
    }
    
    // Adjust each joint in sequence (from end effector back to base)
    // Joint order: Elbow -> Front -> Lateral -> Twist
    
    // 1. Adjust ELBOW
    float delta_elbow = atan2(error_y, sqrt(error_x*error_x + error_z*error_z)) * 180.0 / PI;
    result.elbow += delta_elbow * 0.5; // damping factor
    result.elbow = constrain(result.elbow, 30, 90);
    
    // 2. Adjust FRONT (forward/up rotation)
    current_pos = forwardKinematics(result);
    error_z = target_z - current_pos.z;
    error_y = target_y - current_pos.y;
    float delta_front = atan2(error_z, -error_y) * 180.0 / PI;
    result.front += delta_front * 0.3;
    result.front = constrain(result.front, 0, 35);
    
    // 3. Adjust LATERAL (side raise)
    current_pos = forwardKinematics(result);
    error_x = target_x - current_pos.x;
    error_y = target_y - current_pos.y;
    float delta_lateral = atan2(error_x, -error_y) * 180.0 / PI;
    result.lateral += delta_lateral * 0.3;
    result.lateral = constrain(result.lateral, 0, 35);
    
    // 4. Adjust TWIST (horizontal rotation)
    current_pos = forwardKinematics(result);
    error_x = target_x - current_pos.x;
    error_z = target_z - current_pos.z;
    float delta_twist = atan2(error_x, error_z) * 180.0 / PI;
    result.twist += delta_twist * 0.4;
    result.twist = constrain(result.twist, 
      potToAngleTwist(shoulderTwistMin), 
      potToAngleTwist(shoulderTwistMax));
  }
  
  Serial.println("=== CCD did not fully converge ===");
  Serial.printf("Final angles: Lat=%.1f° Front=%.1f° Twist=%.1f° Elbow=%.1f°\n",
    result.lateral, result.front, result.twist, result.elbow);
  
  // Calculate final error
  Vector3 final_pos = forwardKinematics(result);
  float final_error = sqrt(
    pow(target_x - final_pos.x, 2) +
    pow(target_y - final_pos.y, 2) +
    pow(target_z - final_pos.z, 2)
  );
  
  Serial.printf("Final error: %.3fm (%.1fcm)\n", final_error, final_error * 100);
  
  // Always return true - we'll get as close as we can
  return true;
}

// ======================= MOVEMENT CONTROL =======================
const float ANGLE_TOLERANCE = 2.0;

void moveToTargetAngles() {
  updateCurrentAngles();
  
  bool anyMoving = false;
  
  // Lateral movement
  float latError = targetAngles.lateral - currentAngles.lateral;
  if (fabs(latError) > ANGLE_TOLERANCE) {
    anyMoving = true;
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
    anyMoving = true;
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
    anyMoving = true;
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
    anyMoving = true;
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
  
  // Debug: Print errors every second
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    lastDebug = millis();
    Serial.printf("Errors: Lat=%.1f° Front=%.1f° Twist=%.1f° Elbow=%.1f° | Moving=%s\n",
      latError, frontError, twistError, elbowError, anyMoving ? "YES" : "NO");
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

  if (cmd.startsWith("goto ")) {
    cmd.remove(0, 5);
    
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.lastIndexOf(',');
    
    if (comma1 > 0 && comma2 > comma1) {
      float x = cmd.substring(0, comma1).toFloat();
      float y = cmd.substring(comma1 + 1, comma2).toFloat();
      float z = cmd.substring(comma2 + 1).toFloat();
      
      Serial.printf("Moving to target: X=%.3f Y=%.3f Z=%.3f\n", x, y, z);
      sendDebugStatus("IK: Starting solve...");
      
      // Always attempt to solve, even if seemingly unreachable
      solveCCD(x, y, z, targetAngles);
      
      ikTarget.x = x;
      ikTarget.y = y;
      ikTarget.z = z;
      ikTarget.active = true;
      activeCommand = "ik_move";
      
      char statusMsg[100];
      snprintf(statusMsg, sizeof(statusMsg), 
        "IK: Target angles: Lat=%.1f Front=%.1f Twist=%.1f Elbow=%.1f",
        targetAngles.lateral, targetAngles.front, targetAngles.twist, targetAngles.elbow);
      sendDebugStatus(statusMsg);
    } else {
      sendDebugStatus("IK: FAILED - Invalid goto format!");
    }
  }
  else if (cmd == "stop_all") {
    activeCommand = "";
    ikTarget.active = false;
    stopAllMotors();
  }
  else {
    activeCommand = cmd;
    ikTarget.active = false;
  }
}

// ======================= SEND POT VALUES BACK =======================
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 500;

void sendPotValues() {
  updateCurrentAngles();
  
  // Calculate current hand position using forward kinematics
  Vector3 handPos = forwardKinematics(currentAngles);
  
  char msg[200];
  snprintf(msg, sizeof(msg), "Status:%s | Pos:(%.3f,%.3f,%.3f) | Ang: L%.1f F%.1f T%.1f E%.1f | Pot:%d,%d,%d,%d",
    activeCommand.c_str(),
    handPos.x, handPos.y, handPos.z,
    currentAngles.lateral, currentAngles.front, currentAngles.twist, currentAngles.elbow,
    analogRead(shoulderLatPot), analogRead(shoulderFrontPot), 
    analogRead(shoulderTwistPot), analogRead(bicepPot));

  esp_err_t res = esp_now_send(senderMAC, (uint8_t*)msg, strlen(msg) + 1);
  if (res != ESP_OK) {
    uint8_t bcast[6];
    memset(bcast, 0xFF, 6);
    esp_now_send(bcast, (uint8_t*)msg, strlen(msg) + 1);
  }
  
  // Also print to serial for debugging
  Serial.printf("Hand Position: (%.3f, %.3f, %.3f)\n", handPos.x, handPos.y, handPos.z);
}

void sendDebugStatus(const char* msg) {
  Serial.println(msg);
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

  Serial.println("CCD IK Controller ready!");
  Serial.println("Send 'goto x,y,z' to move to position (meters, relative to eyes)");
}

// ======================= LOOP =======================
void loop() {
  unsigned long now = millis();

  if (activeCommand == "ik_move" && ikTarget.active) {
    moveToTargetAngles();
    
    updateCurrentAngles();
    if (fabs(targetAngles.lateral - currentAngles.lateral) < ANGLE_TOLERANCE &&
        fabs(targetAngles.front - currentAngles.front) < ANGLE_TOLERANCE &&
        fabs(targetAngles.twist - currentAngles.twist) < ANGLE_TOLERANCE &&
        fabs(targetAngles.elbow - currentAngles.elbow) < ANGLE_TOLERANCE) {
      Serial.println("Target reached!");
      sendDebugStatus("IK: Target REACHED!");
      ikTarget.active = false;
      activeCommand = "";
      stopAllMotors();
    }
  }
  else if (activeCommand != "" && activeCommand != "ik_move") {
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

  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    sendPotValues();
  }
  
  delay(1);
}