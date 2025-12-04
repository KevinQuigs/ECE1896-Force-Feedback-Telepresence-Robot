#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ========== REPLACE WITH RECEIVER MAC ==========
// uint8_t receiverMAC[] = {0xCC, 0xDB, 0xA7, 0x96, 0x60, 0x1C};
uint8_t receiverMAC[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14}; // mICRO ARM

// ----- RECEIVE CALLBACK (pot readings arriving from receiver) -----
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  String msg = String((char*)data);
  msg.trim();
  Serial.print("Received pot data: ");
  Serial.println(msg);
}

// ----- SEND CALLBACK -----
void OnSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.print("This device MAC: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while(true) delay(1000);
  }

  esp_now_register_send_cb(OnSent);
  esp_now_register_recv_cb(onDataRecv);

  // add receiver as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    // keep going — peer add might already exist
  }

  Serial.println("Sender ready. Type movement commands (e.g., lateral_up) and press Enter.");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      esp_err_t r = esp_now_send(receiverMAC, (uint8_t*)cmd.c_str(), cmd.length() + 1);
      Serial.print("Sent: ");
      Serial.print(cmd);
      Serial.print("  (esp_now_send result=");
      Serial.print(r);
      Serial.println(")");
    }
  }

  // nothing else to do on sender
  delay(10);
}
