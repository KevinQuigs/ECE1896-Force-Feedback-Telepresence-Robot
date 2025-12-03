/*
 * ESP32 Robot Controller
 * Version: Iteration 3 -
 *   -Receives tracking data via ESP-NOW from another ESP32
 *   -Sends force feedback values back via ESP-NOW
 *   -Controls servos for robotic neck based on received head tracking data
 */
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

String incoming_tracking_data = "";  // Buffer for incoming tracking data

// MAC address of the ESP32 that sends tracking data
uint8_t espTracker_mac[] = {0xCC, 0xDB, 0xA7, 0x9A, 0xDF, 0x1C}; // UPDATE WITH ACTUAL MAC

const int BAUD_RATE = 115200;

const int PIN_LEFT_SERVO = 25;
const int PIN_RIGHT_SERVO = 26;
const int PIN_YAW_SERVO = 27;

Servo SERVO_LEFT;
Servo SERVO_RIGHT;
Servo SERVO_YAW;

// DEBUG: Set to true to enable serial debug output
const bool DEBUG_MODE = true;

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

struct ForceSensorData {
    float thumb;
    float index;
    float middle;
    float ring;
    float pinky;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

TrackingData trackingData;
ForceSensorData forceSensorData;

unsigned long lastDebugTime = 0;

// Debug counters
unsigned long messagesReceived = 0;
bool newTrackingDataAvailable = false;

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(BAUD_RATE);
    delay(1000);

    // ESP NOW SETUP
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("ESP-Neck READY (receiving via ESP-NOW)");
    Serial.print("My MAC Address: ");
    Serial.println(WiFi.macAddress());
    
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
        Serial.println("ESP32 Robot Controller - ESP-NOW Mode");
        Serial.println("=================================");
        Serial.println("Waiting for tracking data via ESP-NOW...");
        Serial.println("=================================");
    }
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    // Process new tracking data if available
    if (newTrackingDataAvailable) {
        parseTrackingDataString(incoming_tracking_data);
        newTrackingDataAvailable = false;
        
        if (DEBUG_MODE) {
            Serial.println("[DEBUG] Processed new tracking data");
        }
    }
    
    // Debug output every second
    if (DEBUG_MODE) {
        unsigned long currentTime = millis();
        if (currentTime - lastDebugTime >= 1000) {
            lastDebugTime = currentTime;
            
            Serial.print("[DEBUG] Total msgs: ");
            Serial.print(messagesReceived);
            Serial.print(" | Head P/Y/R: ");
            Serial.print(trackingData.headRotPitch);
            Serial.print("/");
            Serial.print(trackingData.headRotYaw);
            Serial.print("/");
            Serial.println(trackingData.headRotRoll);
        }
    }
    
    // Send force sensor data back via ESP-NOW
    //sendForceSensorData();
    
    // Control servos
    updateNeckControl();
    
    delay(10);  // Small delay to prevent overwhelming the loop
}

// =============================================================================
// PARSE TRACKING DATA FROM STRING
// =============================================================================

void parseTrackingDataString(String data) {
    data.trim();
    
    int index1 = data.indexOf(',');
    trackingData.fingerThumb = (data.substring(0, index1).toFloat());
    
    int index2 = data.indexOf(',', index1 + 1); 
    trackingData.fingerIndex = (data.substring(index1 + 1, index2).toFloat());

    int index3 = data.indexOf(',', index2 + 1); 
    trackingData.fingerMiddle = (data.substring(index2 + 1, index3).toFloat());

    int index4 = data.indexOf(',', index3 + 1); 
    trackingData.fingerRing = (data.substring(index3 + 1, index4).toFloat());

    int index5 = data.indexOf(',', index4 + 1); 
    trackingData.fingerPinky = (data.substring(index4 + 1, index5).toFloat());

    int index6 = data.indexOf(',', index5 + 1); 
    trackingData.handPosX = (data.substring(index5 + 1, index6).toFloat());

    int index7 = data.indexOf(',', index6 + 1); 
    trackingData.handPosY = (data.substring(index6 + 1, index7).toFloat());

    int index8 = data.indexOf(',', index7 + 1); 
    trackingData.handPosZ = (data.substring(index7 + 1, index8).toFloat());

    int index9 = data.indexOf(',', index8 + 1); 
    trackingData.handRotPitch = (data.substring(index8 + 1, index9).toFloat());

    int index10 = data.indexOf(',', index9 + 1); 
    trackingData.handRotYaw = (data.substring(index9 + 1, index10).toFloat());

    int index11 = data.indexOf(',', index10 + 1); 
    trackingData.handRotRoll = (data.substring(index10 + 1, index11).toFloat());

    int index12 = data.indexOf(',', index11 + 1); 
    trackingData.headRotPitch = (data.substring(index11 + 1, index12).toFloat());

    int index13 = data.indexOf(',', index12 + 1); 
    trackingData.headRotYaw = (data.substring(index12 + 1, index13).toFloat());

    trackingData.headRotRoll = (data.substring(index13 + 1).toFloat());
    
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
        
        if (left_motor_ang < 30) {
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

        if (right_motor_ang < 30) {
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
    left_motor_ang = constrain(left_motor_ang, 30, 180);
    right_motor_ang = constrain(right_motor_ang, 30, 180);

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
    float des_ang = constrain(desired_angle, -26, 15);
    return (int)solveForX(des_ang);
}

float solveForX(float des_ang) {
    float x = 0;

    if (des_ang <= -24.18) {
        x = (des_ang - 5.4761) / 1.508;
    } else if (des_ang <= -21.49) {
        x = (des_ang + 29.645) / 0.1345;
    } else if (des_ang <= -19.32) {
        x = (des_ang + 34.51) / 0.217;
    } else if (des_ang <= 1.45) {
        x = (des_ang + 42.655) / 0.3428;
    } else if (des_ang <= 7.33) {
        x = (des_ang + 36.947) / 0.294;
    } else if (des_ang <= 10.62) {
        x = (des_ang + 42.02) / 0.329;
    } else {
        x = (des_ang + 21.082) / 0.1985;
    }

    x = constrain(x, 30, 180);
    return x;
}

int calculateRightMotorAngle(int leftMotorAngleRef) {
    int angle = 180 - leftMotorAngleRef;
    return constrain(angle, 0, 150);
}

int calculateAngleForYaw(float desired_angle) {
    return constrain((int)desired_angle, 1, 179);
}

float solveForDisplacementRoll(float desired_angle) {
    float disp = 0;

    if (desired_angle >= 24) {
        disp = (desired_angle - 3.064) / 0.8462;
        disp = constrain(disp, -150, 150);
    } else if (desired_angle >= 18.54) {
        disp = (desired_angle - 6.425) / 0.1365;
    } else if (desired_angle >= -15.09) {
        disp = (desired_angle - 0.2933) / 0.2933;
    } else if (desired_angle >= -21.29) {
        disp = (desired_angle + 4.1333) / 0.155;
    } else {
        disp = (desired_angle + 5.9685) / 0.1385;
        disp = constrain(disp, -150, 150);
    }

    return disp / 2;
}

float radToDeg(float rad) {
    return (rad * 180 / PI);
}

// =============================================================================
// ESP-NOW CALLBACK
// =============================================================================

// Callback when receiving data via ESP-NOW
void onReceive(const esp_now_recv_info *info, const uint8_t *incoming, int len) {
    String receivedData = String((char*)incoming);
    
    // Check if this is tracking data (contains commas) or force sensor data
    if (receivedData.indexOf(',') >= 0 && receivedData.indexOf("FORCE") < 0) {
        // This is tracking data
        incoming_tracking_data = receivedData;
        newTrackingDataAvailable = true;
        messagesReceived++;
        
        if (DEBUG_MODE) {
            Serial.println("[ESP-NOW] Received tracking data: " + receivedData.substring(0, 50) + "...");
        }
    } else if (receivedData.startsWith("T") || receivedData.indexOf("f.") >= 0) {
        // This is force sensor feedback (format: "Tf.2,If.2,Mf.2,Rf.1,Pf.2")
        decodeForceSensors(receivedData);
        
        if (DEBUG_MODE) {
            Serial.println("[ESP-NOW] Received force data: " + receivedData);
        }
    }
}

//data coming in in form of "Tf.2,If.2,Mf.2,Rf.1,Pf.2"
void decodeForceSensors(String data) {
    int index1 = data.indexOf(',');
    forceSensorData.thumb = (data.substring(0, index1).toFloat());
    int index2 = data.indexOf(',', index1 + 1); 
    forceSensorData.index = (data.substring(index1 + 1, index2).toFloat());
    int index3 = data.indexOf(',', index2 + 1);
    forceSensorData.middle = (data.substring(index2 + 1, index3).toFloat());
    int index4 = data.indexOf(',', index3 + 1);
    forceSensorData.ring = (data.substring(index3 + 1, index4).toFloat());
    forceSensorData.pinky = (data.substring(index4 + 1).toFloat());
}

void sendForceSensorData() {
    // Format: "FORCE:thumb,index,middle,ring,pinky"
    String forceData = "FORCE:" + 
                       String(forceSensorData.thumb, 2) + "," +
                       String(forceSensorData.index, 2) + "," +
                       String(forceSensorData.middle, 2) + "," +
                       String(forceSensorData.ring, 2) + "," +
                       String(forceSensorData.pinky, 2);
    
    // Send back to tracker ESP via ESP-NOW
    uint8_t data[100];
    forceData.getBytes(data, forceData.length() + 1);
    esp_err_t result = esp_now_send(espTracker_mac, data, forceData.length() + 1);
    
    // Also output to Serial for debugging
    if (DEBUG_MODE) {
        Serial.println(forceData);
        if (result != ESP_OK) {
            Serial.println("[ERROR] Failed to send force data");
        }
    }
}