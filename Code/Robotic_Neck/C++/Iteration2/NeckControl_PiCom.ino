/*
 * ESP32 Robot Controller
 * Version: Iteration 2 -
 *   -Communicates serially with Raspberry Pi to receive tracking data and send force feedback values
 *   -Controls servos for robotic neck based on received head tracking data
 *   -Uses ESPNOW for sharing data to other ESP32 modules on the robot
 *
 * **Added debugging to diagnose communication issues
 */
#include <ESP32Servo.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

const int BAUD_RATE = 115200;
const int SEND_INTERVAL = 50;

const int PIN_LEFT_SERVO = 25;
const int PIN_RIGHT_SERVO = 26;
const int PIN_YAW_SERVO = 27;

Servo SERVO_LEFT;
Servo SERVO_RIGHT;
Servo SERVO_YAW;

// Message type identifier
const uint8_t MSG_TYPE_TRACKING = 0x01;

// Binary message size: 1 byte type + 14 floats (4 bytes each) = 57 bytes
const int TRACKING_MSG_SIZE = 57;

// DEBUG: Set to true to enable serial debug output
const bool DEBUG_MODE = false;

// =============================================================================
// DATA STRUCTURES
// =============================================================================

struct TrackingData {
    float fingerThumb;
    float fingerIndex;
    float fingerMiddle;
    float fingerRing;
    float fingerPinky;
    
    float handPosX;
    float handPosY;
    float handPosZ;
    
    float handRotPitch;
    float handRotYaw;
    float handRotRoll;
    
    float headRotPitch;
    float headRotYaw;
    float headRotRoll;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

TrackingData trackingData;

unsigned long lastSendTime = 0;
unsigned long lastDebugTime = 0;
uint8_t serialBuffer[TRACKING_MSG_SIZE];
int bufferIndex = 0;

// Debug counters
unsigned long bytesReceived = 0;
unsigned long messagesReceived = 0;
unsigned long lastBytesReceived = 0;

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(BAUD_RATE);
    delay(1000);
    
    memset(&trackingData, 0, sizeof(TrackingData));
    
    // Set default head position
    trackingData.headRotPitch = 0;
    trackingData.headRotYaw = 90;
    trackingData.headRotRoll = 0;

    SERVO_LEFT.attach(PIN_LEFT_SERVO, 500, 2500); 
    SERVO_RIGHT.attach(PIN_RIGHT_SERVO, 500, 2500); 
    SERVO_YAW.attach(PIN_YAW_SERVO, 500, 2500); 
    
    // Move to center position on startup
    SERVO_LEFT.write(90);
    SERVO_RIGHT.write(90);
    SERVO_YAW.write(90);
    
    if (DEBUG_MODE) {
        Serial.println("=================================");
        Serial.println("ESP32 Robot Controller - DEBUG");
        Serial.println("=================================");
        Serial.println("Waiting for tracking data...");
        Serial.print("Expected message size: ");
        Serial.println(TRACKING_MSG_SIZE);
        Serial.println("=================================");
    }
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    // Read tracking data from Pi
    readTrackingData();
    
    // Debug output every second
    if (DEBUG_MODE) {
        unsigned long currentTime = millis();
        if (currentTime - lastDebugTime >= 1000) {
            lastDebugTime = currentTime;
            
            unsigned long newBytes = bytesReceived - lastBytesReceived;
            lastBytesReceived = bytesReceived;
            
            Serial.print("[DEBUG] Bytes/sec: ");
            Serial.print(newBytes);
            Serial.print(" | Total msgs: ");
            Serial.print(messagesReceived);
            Serial.print(" | Buffer idx: ");
            Serial.print(bufferIndex);
            Serial.print(" | Head P/Y/R: ");
            Serial.print(trackingData.headRotPitch);
            Serial.print("/");
            Serial.print(trackingData.headRotYaw);
            Serial.print("/");
            Serial.println(trackingData.headRotRoll);
        }
    }
    
    // Control servos
    updateNeckControl();
}

// =============================================================================
// RECEIVE TRACKING DATA FROM PI
// =============================================================================

void readTrackingData() {
    while (Serial.available() > 0) {
        uint8_t inByte = Serial.read();
        bytesReceived++;
        
        // Check for message start (message type byte)
        if (bufferIndex == 0) {
            if (inByte == MSG_TYPE_TRACKING) {
                serialBuffer[bufferIndex++] = inByte;
                if (DEBUG_MODE) {
                    // Uncomment for very verbose debugging
                    // Serial.println("[DEBUG] Message start detected");
                }
            } else {
                // Debug: Show unexpected bytes
                if (DEBUG_MODE && inByte != 0x00 && inByte != '\n' && inByte != '\r') {
                    Serial.print("[DEBUG] Unexpected byte: 0x");
                    Serial.println(inByte, HEX);
                }
            }
        } else {
            // Collecting message bytes
            serialBuffer[bufferIndex++] = inByte;
            
            // Check if we have complete message
            if (bufferIndex >= TRACKING_MSG_SIZE) {
                parseTrackingData();
                messagesReceived++;
                bufferIndex = 0;
                
                if (DEBUG_MODE) {
                    Serial.print("[DEBUG] Message parsed! Head: P=");
                    Serial.print(trackingData.headRotPitch);
                    Serial.print(" Y=");
                    Serial.print(trackingData.headRotYaw);
                    Serial.print(" R=");
                    Serial.println(trackingData.headRotRoll);
                }
            }
        }
    }
}

void parseTrackingData() {
    int offset = 1;
    
    // Parse finger rotations
    memcpy(&trackingData.fingerThumb, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.fingerIndex, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.fingerMiddle, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.fingerRing, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.fingerPinky, &serialBuffer[offset], 4); offset += 4;
    
    // Parse hand position
    memcpy(&trackingData.handPosX, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.handPosY, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.handPosZ, &serialBuffer[offset], 4); offset += 4;
    
    // Parse hand rotation
    memcpy(&trackingData.handRotPitch, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.handRotYaw, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.handRotRoll, &serialBuffer[offset], 4); offset += 4;
    
    // Parse head rotation
    memcpy(&trackingData.headRotPitch, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.headRotYaw, &serialBuffer[offset], 4); offset += 4;
    memcpy(&trackingData.headRotRoll, &serialBuffer[offset], 4); offset += 4;
    
    // DEBUG: Print ALL parsed values
    if (DEBUG_MODE) {
        Serial.println("=== FULL DATA DUMP ===");
        Serial.print("Fingers: ");
        Serial.print(trackingData.fingerThumb); Serial.print(", ");
        Serial.print(trackingData.fingerIndex); Serial.print(", ");
        Serial.print(trackingData.fingerMiddle); Serial.print(", ");
        Serial.print(trackingData.fingerRing); Serial.print(", ");
        Serial.println(trackingData.fingerPinky);
        
        Serial.print("Hand Pos: ");
        Serial.print(trackingData.handPosX); Serial.print(", ");
        Serial.print(trackingData.handPosY); Serial.print(", ");
        Serial.println(trackingData.handPosZ);
        
        Serial.print("Hand Rot: ");
        Serial.print(trackingData.handRotPitch); Serial.print(", ");
        Serial.print(trackingData.handRotYaw); Serial.print(", ");
        Serial.println(trackingData.handRotRoll);
        
        Serial.print("Head Rot: ");
        Serial.print(trackingData.headRotPitch); Serial.print(", ");
        Serial.print(trackingData.headRotYaw); Serial.print(", ");
        Serial.println(trackingData.headRotRoll);
        Serial.println("======================");
    }
}

// =============================================================================
// ROBOT CONTROL
// =============================================================================

void updateNeckControl() {
    float pitch_ang = calculateAngleForPitch(trackingData.headRotPitch);
    float roll_disp = solveForDisplacementRoll(trackingData.headRotRoll);
    float left_motor_ang = pitch_ang;
    float right_motor_ang = pitch_ang;

    // Apply roll displacement
    if (roll_disp > 0) {
        left_motor_ang = pitch_ang - roll_disp;
        right_motor_ang = pitch_ang + roll_disp;
        
        if (left_motor_ang < 0) {
            float new_disp = roll_disp + left_motor_ang;
            left_motor_ang = pitch_ang - new_disp;
            right_motor_ang = pitch_ang + new_disp;
        } else if (right_motor_ang > 180) {
            float new_disp = roll_disp - 180 + right_motor_ang;
            right_motor_ang = pitch_ang + new_disp;
            left_motor_ang = pitch_ang - new_disp;
        }
    } else {
        left_motor_ang = pitch_ang - roll_disp;
        right_motor_ang = pitch_ang + roll_disp;

        if (right_motor_ang < 0) {
            float new_disp = roll_disp - right_motor_ang;
            right_motor_ang = pitch_ang + new_disp;
            left_motor_ang = pitch_ang - new_disp;
        } else if (left_motor_ang > 180) {
            float new_disp = roll_disp - 180 + left_motor_ang;
            left_motor_ang = pitch_ang - new_disp;
            right_motor_ang = pitch_ang + new_disp;
        }
    }

    // Constrain values
    left_motor_ang = constrain(left_motor_ang, 0, 180);
    right_motor_ang = constrain(right_motor_ang, 0, 180);

    int right_final = calculateRightMotorAngle((int)right_motor_ang);
    int left_final = (int)left_motor_ang;
    int yaw_final = calculateAngleForYaw(trackingData.headRotYaw);

    // DEBUG: Print calculated servo angles
    if (DEBUG_MODE) {
        static unsigned long lastServoPrint = 0;
        if (millis() - lastServoPrint >= 500) {  // Print every 500ms
            lastServoPrint = millis();
            Serial.println("--- SERVO DEBUG ---");
            Serial.print("Input Head P/Y/R: ");
            Serial.print(trackingData.headRotPitch); Serial.print(" / ");
            Serial.print(trackingData.headRotYaw); Serial.print(" / ");
            Serial.println(trackingData.headRotRoll);
            
            Serial.print("pitch_ang: "); Serial.println(pitch_ang);
            Serial.print("roll_disp: "); Serial.println(roll_disp);
            
            Serial.print("Servo angles -> Left: "); Serial.print(left_final);
            Serial.print(" | Right: "); Serial.print(right_final);
            Serial.print(" | Yaw: "); Serial.println(yaw_final);
            Serial.println("-------------------");
        }
    }

    SERVO_RIGHT.write(right_final);
    SERVO_LEFT.write(left_final);
    SERVO_YAW.write(yaw_final);
}

int calculateAngleForPitch(float desired_angle) {
    float des_ang = constrain(desired_angle, -16, 26);
    return (int)solveForX(des_ang);
}

float solveForX(float des_ang) {
    float x = 0;

    if (des_ang <= -14.6) {
        x = (des_ang + 16.133) / 0.07;
    } else if (des_ang <= -11.44) {  // FIXED: was wrong comparison
        x = (des_ang + 16.804) / 0.1369;
    } else if (des_ang <= 13.68) {   // FIXED
        x = (des_ang + 24.682) / 0.3125;
    } else if (des_ang <= 19.62) {   // FIXED
        x = (des_ang + 21.957) / 0.297;
    } else if (des_ang <= 24.04) {   // FIXED
        x = (des_ang + 11.227) / 0.221;
    } else {
        x = (des_ang - 8.6733) / 0.096;
    }

    return x;
}

int calculateRightMotorAngle(int leftMotorAngleRef) {
    int angle = 185 - leftMotorAngleRef;
    return constrain(angle, 0, 180);
}

int calculateAngleForYaw(float desired_angle) {
    return constrain((int)desired_angle, 1, 179);
}

float solveForDisplacementRoll(float desired_angle) {
    float disp = 0;

    // FIXED: All comparisons were wrong in C++
    if (desired_angle >= 23.42) {
        disp = (desired_angle + 5.6937) / 1.2678;
        disp = constrain(disp, -180, 180);
    } else if (desired_angle >= 19.96) {       // FIXED
        disp = (desired_angle + 1.353) / 1.1716;
    } else if (desired_angle >= -16.61) {      // FIXED
        disp = (desired_angle - 0.9351) / (-0.2079);
    } else if (desired_angle >= -20.78) {      // FIXED
        disp = (desired_angle + 4.4055) / (-0.1426);
    } else if (desired_angle >= -23.25) {      // FIXED
        disp = (desired_angle + 1.9336) / (0.8106);
    } else {
        disp = (desired_angle + 3.6836) / 0.8152;
        disp = constrain(disp, -180, 180);
    }

    return disp / 2;
}

float radToDeg(float rad) {
    return (rad * 180 / PI);
}