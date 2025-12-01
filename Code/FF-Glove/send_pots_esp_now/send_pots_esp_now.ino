#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ADC pins
#define POT_THUMB 36
#define POT_INDEX 39
#define POT_MIDDLE 34
#define POT_RING 12
#define POT_PINKY 13

String feedback = "";

// MAC ADDRESSES FOR ESP-B & ESP-C
uint8_t espB_mac[] = {0xCC, 0xDB, 0xA7, 0x90, 0xB7, 0xA4};  // HAND ESP
// uint8_t espB_mac[] = {0xF0, 0x24, 0xF9, 0x59, 0x8E, 0x1C};  // SETH ESP
uint8_t espC_mac[] = {0xCC, 0xDB, 0xA7, 0x96, 0x60, 0x1C};

// Callback when receiving feedback from B or C (haptic)
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

// Map raw pot to angle
int convert_pot2ang(int pot_val, int desired_min, int desired_max, int measured_min, int measured_max){
    if (pot_val > measured_max) pot_val = measured_max;
    else if (pot_val < measured_min) pot_val = measured_min;

    float actual_range = measured_max - measured_min;
    float desired_range = desired_max - desired_min;

    float normalized_value = float(pot_val - measured_min) / actual_range;

    return int(normalized_value * desired_range + desired_min);
}

void setup() {
  Serial.begin(115200);

  pinMode(POT_THUMB, INPUT);
  pinMode(POT_INDEX, INPUT);
  pinMode(POT_MIDDLE, INPUT);
  pinMode(POT_RING, INPUT);
  pinMode(POT_PINKY, INPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  addPeer(espB_mac);
  addPeer(espC_mac);

  Serial.println("ESP-NOW READY (broadcasting to B & C)");
}

void loop() {

  // 1) Read potentiometer values and convert to angles
  int thumb = 180 - convert_pot2ang(analogRead(POT_THUMB), 0, 180, 0, 2270);
  int index = 180 - convert_pot2ang(analogRead(POT_INDEX), 0, 180, 0, 1780);
  int middle = 180 - convert_pot2ang(analogRead(POT_MIDDLE), 0, 180, 2300, 4095);
  int ring   = 180 - convert_pot2ang(analogRead(POT_RING),   0, 180, 1320, 3100);
  int pinky  = 180 - convert_pot2ang(analogRead(POT_PINKY),  0, 180, 75, 1800);

  // 2) Format as comma-separated string
  String data = String(thumb) + "," + String(index) + "," + String(middle) + "," + String(ring) + "," + String(pinky);

  // 3) Broadcast to both B & C via ESP-NOW
  uint8_t dataBytes[200];
  data.getBytes(dataBytes, data.length() + 1);

  esp_now_send(espB_mac, dataBytes, data.length() + 1);
  esp_now_send(espC_mac, dataBytes, data.length() + 1);

  // 4) Also send to Serial for debugging
  Serial.println(data);

  // 5) If feedback from B or C → send to Serial
  if (feedback.length() > 0) {
    Serial.print("FB ");
    Serial.println(feedback);
    feedback = "";
  }

  delay(20);
}