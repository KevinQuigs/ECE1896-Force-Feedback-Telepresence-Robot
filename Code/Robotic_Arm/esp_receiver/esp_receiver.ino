#include <WiFi.h>
#include <esp_now.h>

#define ESPNOW_WIFI_CHANNEL 6

// Communicating with ESP 2
uint8_t peerMac[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14};

// Callback for received data
void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  Serial.print("Received from ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" → ");
  for (int i = 0; i < len; i++) {
        Serial.print(incomingData[i]);
        Serial.print(" ");
    }
    Serial.println();

  // Sends message back
  const char msg[] = "Hello from ESP 2!";
  esp_err_t result = esp_now_send(peerMac, (uint8_t *)msg, strlen(msg));
  if (result == ESP_OK) {
    Serial.println("Message sent!");
  } else {
    Serial.println("Send failed");
  }
  
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    ESP.restart();
  }

  // Register receive callback with correct signature
  esp_now_register_recv_cb(onReceive);

  Serial.println("ESP-NOW ready to send and receive");
}

void loop() {
  
}
