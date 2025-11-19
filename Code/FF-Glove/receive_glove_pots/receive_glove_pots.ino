#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

String incomingData = "";

void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  incomingData = String((char*)data);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  Serial.println("ESP-A READY (receiving glove data)");
}

void loop() {
  if (incomingData.length() > 0) {
    Serial.println(incomingData);
    incomingData = "";
  }
  
  delay(3000);
}
