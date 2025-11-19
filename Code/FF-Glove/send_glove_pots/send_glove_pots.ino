#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Replace with REAL ESP-A MAC
uint8_t espA_mac[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14};

#define POT_THUMB 35
#define POT_INDEX 34
#define POT_MIDDLE 14
#define POT_RING 12
#define POT_PINKY 13

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed");
    return;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, espA_mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  esp_now_register_send_cb(onSent);

  pinMode(POT_THUMB, INPUT);
  pinMode(POT_INDEX, INPUT);
  pinMode(POT_MIDDLE, INPUT);
  pinMode(POT_RING, INPUT);
  pinMode(POT_PINKY, INPUT);
}

void loop() {
  int t = analogRead(POT_THUMB);
  int i = analogRead(POT_INDEX);
  int m = analogRead(POT_MIDDLE);
  int r = analogRead(POT_RING);
  int p = analogRead(POT_PINKY);

  char msg[100];
  sprintf(msg, "%d,%d,%d,%d,%d", t, i, m, r, p);

  esp_now_send(espA_mac, (uint8_t*)msg, strlen(msg) + 1);

  delay(20); // ~50 Hz
}
