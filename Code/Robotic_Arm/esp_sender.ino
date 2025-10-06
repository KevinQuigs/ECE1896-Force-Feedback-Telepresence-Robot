#include <WiFi.h>
#include <esp_now.h>

#define ESPNOW_WIFI_CHANNEL 6

bool lastState = HIGH;
uint8_t fingerPositions[5];

// Communicating with ESP 1
uint8_t peerMac[] = {0xCC, 0xDB, 0xA7, 0x9B, 0x5F, 0x14};

// Callback for received data
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Serial.print("Received from ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" → ");
  Serial.write(data, len);
  Serial.println();
}

void setup() {
  // SET UP BUTTONS
  pinMode(0, INPUT);

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
  bool currentState = digitalRead(0);
  
  fingerPositions[0] = 0;
  fingerPositions[1] = 1;
  fingerPositions[2] = 2;
  fingerPositions[3] = 3;
  fingerPositions[4] = 4;


  if (currentState == LOW && lastState == HIGH)
  {
    const char msg[] = "Hello from ESP 1!";
    esp_err_t result = esp_now_send(peerMac, (uint8_t *)fingerPositions, sizeof(fingerPositions));
    if (result == ESP_OK) {
      Serial.println("Sent Finger Angles!");
    } else {
      Serial.println("Failed to send Finger Angles");
    }
  }
  lastState = currentState;
  
}
