#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ======= CHANGE THIS TO SENDER MAC =======
uint8_t senderMAC[] = {0x38, 0x18, 0x2B, 0xEB, 0x93, 0x14};

// ======================= JOINT DEFINITIONS =======================
const int shoulderLatPot = 35;
const int shoulderLatServo1 = 4;
const int shoulderLatServo2 = 16;

const int shoulderFrontPot = 32;
const int shoulderFrontServo1 = 2;
const int shoulderFrontServo2 = 15;

const int shoulderTwistPot = 34;
const int shoulderTwistServo1 = 5;
const int shoulderTwistServo2 = 17;

const int bicepPot = 33;
const int bicepServo1 = 12;
const int bicepServo2 = 13;

// ======================= POT LIMITS =======================
int shoulderLatMin = 1550;   // ~0° (actually slightly raised?)
int shoulderLatMax = 1950;   // ~35° out to right

int shoulderFrontMin = 250;  // (lower than neutral)
int shoulderFrontMax = 1000; // ~35° forward/up
// Neutral at 550

int shoulderTwistMin = 150;  // (rotated left)
int shoulderTwistMax = 4000; // (rotated right)
// Neutral at 1000

int bicepMin = 50;           // 30° (straightest possible)
int bicepMax = 1800;         // 90° (bent)

// ======================= ACTIVE COMMAND =======================
String activeCommand = ""; // stores currently active movement

// ======================= MOVEMENT HELPERS =======================
void stopLateral() { digitalWrite(shoulderLatServo1, LOW); digitalWrite(shoulderLatServo2, LOW); }
void stopFront()   { digitalWrite(shoulderFrontServo1, LOW); digitalWrite(shoulderFrontServo2, LOW); }
void stopTwist()   { digitalWrite(shoulderTwistServo1, LOW); digitalWrite(shoulderTwistServo2, LOW); }
void stopBicep()   { digitalWrite(bicepServo1, LOW); digitalWrite(bicepServo2, LOW); }

void moveLateralUp() {
  int p = analogRead(shoulderLatPot);
  if (p < shoulderLatMax) { digitalWrite(shoulderLatServo1, HIGH); digitalWrite(shoulderLatServo2, LOW); }
  else stopLateral();
}

void moveLateralDown() {
  int p = analogRead(shoulderLatPot);
  if (p > shoulderLatMin) { digitalWrite(shoulderLatServo1, LOW); digitalWrite(shoulderLatServo2, HIGH); }
  else stopLateral();
}

void moveFrontUp() {
  int p = analogRead(shoulderFrontPot);
  if (p < shoulderFrontMax) { digitalWrite(shoulderFrontServo1, HIGH); digitalWrite(shoulderFrontServo2, LOW); }
  else stopFront();
}

void moveFrontDown() {
  int p = analogRead(shoulderFrontPot);
  if (p > shoulderFrontMin) { digitalWrite(shoulderFrontServo1, LOW); digitalWrite(shoulderFrontServo2, HIGH); }
  else stopFront();
}

void moveTwistRight() {
  int p = analogRead(shoulderTwistPot);
  if (p < shoulderTwistMax) { digitalWrite(shoulderTwistServo1, HIGH); digitalWrite(shoulderTwistServo2, LOW); }
  else stopTwist();
}

void moveTwistLeft() {
  int p = analogRead(shoulderTwistPot);
  if (p > shoulderTwistMin) { digitalWrite(shoulderTwistServo1, LOW); digitalWrite(shoulderTwistServo2, HIGH); }
  else stopTwist();
}

void moveBicepUp() {
  int p = analogRead(bicepPot);
  if (p < bicepMax) { digitalWrite(bicepServo1, HIGH); digitalWrite(bicepServo2, LOW); }
  else stopBicep();
}

void moveBicepDown() {
  int p = analogRead(bicepPot);
  if (p > bicepMin) { digitalWrite(bicepServo1, LOW); digitalWrite(bicepServo2, HIGH); }
  else stopBicep();
}

// ======================= ESP-NOW RECEIVE CALLBACK =======================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len <= 0) return;

  String cmd = String((char*)data);
  cmd.trim();
  Serial.print("CMD received: ");
  Serial.println(cmd);

  if (cmd == "stop_all") activeCommand = "";
  else activeCommand = cmd;
}

// ======================= SEND POT VALUES BACK =======================
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 500; // ms

void sendPotValues() {
  int lat = analogRead(shoulderLatPot);
  int front = analogRead(shoulderFrontPot);
  int twist = analogRead(shoulderTwistPot);
  int bicep = analogRead(bicepPot);

  char msg[80];
  snprintf(msg, sizeof(msg), "Lat:%d Front:%d Twist:%d Bicep:%d", lat, front, twist, bicep);

  esp_err_t res = esp_now_send(senderMAC, (uint8_t*)msg, strlen(msg) + 1);
  if (res != ESP_OK) {
    uint8_t bcast[6]; memset(bcast, 0xFF, 6);
    esp_now_send(bcast, (uint8_t*)msg, strlen(msg) + 1);
  }
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.print("This device MAC: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while(true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, senderMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Warning: could not add sender as peer (may already exist).");
  }

  pinMode(shoulderLatPot, INPUT);
  pinMode(shoulderFrontPot, INPUT);
  pinMode(shoulderTwistPot, INPUT);
  pinMode(bicepPot, INPUT);

  pinMode(shoulderLatServo1, OUTPUT);
  pinMode(shoulderLatServo2, OUTPUT);
  pinMode(shoulderFrontServo1, OUTPUT);
  pinMode(shoulderFrontServo2, OUTPUT);
  pinMode(shoulderTwistServo1, OUTPUT);
  pinMode(shoulderTwistServo2, OUTPUT);
  pinMode(bicepServo1, OUTPUT);
  pinMode(bicepServo2, OUTPUT);

  stopLateral(); stopFront(); stopTwist(); stopBicep();

  Serial.println("Receiver ready. Waiting for commands...");
}


void loop() {
  unsigned long now = millis();

  // ---- Lateral movement ----
  int latPot = analogRead(shoulderLatPot);
  if (activeCommand == "lateral_up" && latPot < shoulderLatMax)
    digitalWrite(shoulderLatServo1, HIGH), digitalWrite(shoulderLatServo2, LOW);
  else if (activeCommand == "lateral_down" && latPot > shoulderLatMin)
    digitalWrite(shoulderLatServo1, LOW), digitalWrite(shoulderLatServo2, HIGH);
  else stopLateral();

  // ---- Front movement ----
  int frontPot = analogRead(shoulderFrontPot);
  if (activeCommand == "front_up" && frontPot < shoulderFrontMax)
    digitalWrite(shoulderFrontServo1, HIGH), digitalWrite(shoulderFrontServo2, LOW);
  else if (activeCommand == "front_down" && frontPot > shoulderFrontMin)
    digitalWrite(shoulderFrontServo1, LOW), digitalWrite(shoulderFrontServo2, HIGH);
  else stopFront();

  // ---- Twist movement ----
  int twistPot = analogRead(shoulderTwistPot);
  if (activeCommand == "twist_right" && twistPot < shoulderTwistMax)
    digitalWrite(shoulderTwistServo1, HIGH), digitalWrite(shoulderTwistServo2, LOW);
  else if (activeCommand == "twist_left" && twistPot > shoulderTwistMin)
    digitalWrite(shoulderTwistServo1, LOW), digitalWrite(shoulderTwistServo2, HIGH);
  else stopTwist();

  // ---- Bicep movement ----
  int bicepPotVal = analogRead(bicepPot);
  if (activeCommand == "elbow_up" && bicepPotVal < bicepMax)
    digitalWrite(bicepServo1, HIGH), digitalWrite(bicepServo2, LOW);
  else if (activeCommand == "elbow_down" && bicepPotVal > bicepMin)
    digitalWrite(bicepServo1, LOW), digitalWrite(bicepServo2, HIGH);
  else stopBicep();

  // ---- periodic feedback ----
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    sendPotValues();
  }
  
  delay(1); // tiny delay just to let MCU breathe
}

