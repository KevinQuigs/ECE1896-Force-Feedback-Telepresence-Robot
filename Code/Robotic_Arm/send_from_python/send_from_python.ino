#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

String feedback = "";

// MAC ADDRESSES FOR ESP-B & ESP-C
uint8_t espB_mac[] = {0xCC, 0xDB, 0xA7, 0x90, 0xB7, 0xA4};  // HAND ESP
uint8_t espA_mac[] = {0xF0, 0x24, 0xF9, 0x5A, 0xC5, 0x7C};  // NECK ESP
// uint8_t espC_mac[] = {0xCC, 0xDB, 0xA7, 0x96, 0x60, 0x1C}; // ARM ESP
uint8_t espC_mac[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14}; // mICRO ARM


// Callback when receiving feedback from B or C (Hall effect values)
void onReceive(const esp_now_recv_info *info, const uint8_t *incoming, int len) {
  feedback = String((char*)incoming);
  
  // Immediately pass to serial when received
  if (feedback.length() > 0) {
    Serial.println(feedback);
  }
}

void addPeer(uint8_t *mac) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  addPeer(espB_mac);
  addPeer(espC_mac);
  addPeer(espA_mac);

  Serial.println("ESP-A READY (broadcasting to B & C)");
}

void loop() {

  // 1) Read input from Python and broadcast to both B & C
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      uint8_t data[200];
      input.getBytes(data, input.length() + 1);

      esp_now_send(espA_mac, data, input.length() + 1);
      esp_now_send(espB_mac, data, input.length() + 1);
      esp_now_send(espC_mac, data, input.length() + 1);
      
    }
  }
  
  // Feedback is now handled immediately in onReceive callback
}