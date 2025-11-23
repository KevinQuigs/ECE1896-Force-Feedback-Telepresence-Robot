#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t espA_mac[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14};  // CHANGE TO ESP-A MAC

String receivedString = "";

// ==== SERVO SETUP ====
const int thumbPin = 15, indexPin = 2, middlePin = 4, ringPin = 16;
const int pinkyPin = 17, wristFlexPin = 12, wristRotatePin = 13;

Servo thumbF, indexF, middleF, ringF, pinkyF, wristFlex, wristRotate;

Servo* servos[] = {&thumbF, &indexF, &middleF, &ringF, &pinkyF, &wristFlex, &wristRotate};
int pins[] = {thumbPin,indexPin,middlePin,ringPin,pinkyPin,wristFlexPin,wristRotatePin};
const char* tags[] = {"T","I","M","R","P","WF","WR"};
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

  int pos = 0;

  while (pos < input.length()) {
    bool matched = false;

    for (int s = 0; s < NUM_SERVOS; s++) {
      int tagLen = strlen(tags[s]);

      // Finger (F prefix)
      if (s <= 4) {
        if (input.substring(pos, pos + 1 + tagLen) == ("F" + String(tags[s]))) {
          pos += 1 + tagLen;

          int start = pos;
          while (pos < input.length() && isDigit(input[pos])) pos++;
          int angle = input.substring(start, pos).toInt();

          switch (s) {
            case 0: moveThumb(angle); break;
            case 1: moveIndex(angle); break;
            case 2: moveMiddle(angle); break;
            case 3: moveRing(angle); break;
            case 4: movePinky(angle); break;
          }

          sendFeedback("OK F" + String(tags[s]) + angle);
          matched = true;
          break;
        }
      }
      // Wrist
      else {
        if (input.substring(pos, pos + tagLen) == tags[s]) {
          pos += tagLen;

          int start = pos;
          while (pos < input.length() && isDigit(input[pos])) pos++;
          int angle = input.substring(start, pos).toInt();

          if (s == 5) moveWristFlex(angle);
          if (s == 6) moveWristRotate(angle);

          sendFeedback("OK " + String(tags[s]) + angle);
          matched = true;
          break;
        }
      }
    }
    
    if (!matched) pos++;
  }
  
}
