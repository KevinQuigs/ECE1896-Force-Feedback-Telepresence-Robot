void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(0, INPUT);
}


const int BUFFER_SIZE = 50;
char buf[BUFFER_SIZE]; // String array

void loop() {
  // Reading game project
  if (Serial.available() > 0) {
    // int len =  Serial.readBytes(buf, BUFFER_SIZE);
    String data = Serial.readStringUntil('\n');

    if (data == "0") {
      digitalWrite(2, LOW);
      Serial.println("LED OFF");
    } 
    else if (data == "1") {
      digitalWrite(2, HIGH);
      Serial.println("LED ON");
    }

  }


  // Send data to the game

  // Serial.println(digitalRead(0));

  delay(200);




  // digitalWrite(2, HIGH);
  // Serial.println(digitalRead(0));

}
