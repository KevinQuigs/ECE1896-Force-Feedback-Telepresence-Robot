#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t espA_mac[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14};  // CHANGE TO ESP-A MAC

String receivedString = "";

// ==== SERVO SETUP ====
const int thumbPin = 15, indexPin = 2, middlePin = 4, ringPin = 16;
const int pinkyPin = 17, wristFlexPin = 13, wristRotatePin = 12;

Servo thumbF, indexF, middleF, ringF, pinkyF, wristFlex, wristRotate;

Servo* servos[] = {&thumbF, &indexF, &middleF, &ringF, &pinkyF, &wristFlex, &wristRotate};
int pins[] = {thumbPin,indexPin,middlePin,ringPin,pinkyPin,wristFlexPin,wristRotatePin};
const char* f[] = {"T","I","M","R","MP","WY","MR"};
const int NUM_SERVOS = 7;

void sendFeedback(String msg) {
  uint8_t data[100];
  msg.getBytes(data, msg.length() + 1);
  esp_now_send(espA_mac, data, msg.length() + 1);
}

void onReceive(const esp_now_recv_info *info, const uint8_t *incoming, int len) {
  receivedString = String((char*)incoming);
  Serial.println(receivedString);
  
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW FAIL");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, espA_mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // Attach servos
  for (int i = 0; i < NUM_SERVOS; i++)
    servos[i]->attach(pins[i], 500, 2400);

  Serial.println("ESP-B/C READY");
}

// Servo movement functions
void moveThumb(int a){ thumbF.write(constrain(a,0,60)); }
void moveIndex(int a){ indexF.write(constrain(a,0,160)); }
void moveMiddle(int a){ middleF.write(constrain(a,0,160)); }
void moveRing(int a){ ringF.write(180 - constrain(a,20,180)); }
void movePinky(int a){ pinkyF.write(180 - constrain(a,20,180)); }
void moveWristFlex(int a){ wristFlex.write(constrain(a,10,160)); }
void moveWristRotate(int a){ wristRotate.write(constrain(a,10,170)); }

void loop() {

  if (receivedString.length() == 0) {
    return;
  }

  String input = receivedString;
  receivedString = "";
  input.trim();

  // Expected format: "angle1,angle2,angle3,angle4,angle5,angle6,angle7"
  // Order: Thumb, Index, Middle, Ring, Pinky, WristFlex, WristRotate

  int values[NUM_SERVOS];
  int valueCount = 0;
  int pos = 0;

  // Parse comma-separated values
  while (pos < input.length() && valueCount < NUM_SERVOS) {
    int start = pos;
    
    // Read digits (and optional negative sign)
    if (input[pos] == '-') pos++;
    while (pos < input.length() && isDigit(input[pos])) pos++;
    
    if (pos > start) {
      values[valueCount] = input.substring(start, pos).toInt();
      valueCount++;
    }
    
    // Skip comma
    if (pos < input.length() && input[pos] == ',') {
      pos++;
    }
  }

  // Apply values to servos in order
  if (valueCount > 0 && values[0] >= 0) moveThumb(values[0]);
  if (valueCount > 1 && values[1] >= 0) moveIndex(values[1]);
  if (valueCount > 2 && values[2] >= 0) moveMiddle(values[2]);
  if (valueCount > 3 && values[3] >= 0) moveRing(values[3]);
  if (valueCount > 4 && values[4] >= 0) movePinky(values[4]);
  if (valueCount > 5 && values[5] >= 0) moveWristFlex(values[5]);
  if (valueCount > 6 && values[6] >= 0) moveWristRotate(values[6]);

  // Send feedback
  String feedback = "OK ";
  for (int i = 0; i < valueCount; i++) {
    feedback += String(values[i]);
    if (i < valueCount - 1) feedback += ",";
  }
  sendFeedback(feedback);
}
