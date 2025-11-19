#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

String feedback = "";

// MAC ADDRESSES FOR ESP-B & ESP-C
uint8_t espB_mac[] = {0xCC, 0xDB, 0xA7, 0x90, 0xB7, 0xA4}; 
uint8_t espC_mac[] = {0x24, 0x6F, 0x28, 0xDD, 0xEE, 0xFF};

// Callback when receiving feedback from B or C
void onReceive(const esp_now_recv_info *info, const uint8_t *incoming, int len) {
  feedback = String((char*)incoming);
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

      esp_now_send(espB_mac, data, input.length() + 1);
      esp_now_send(espC_mac, data, input.length() + 1);
    }
  }

  // 2) If feedback from B or C → send to Python
  if (feedback.length() > 0) {
    Serial.print("FB ");
    Serial.println(feedback);
    feedback = "";
  }
}
