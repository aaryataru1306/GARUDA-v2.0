const int armStatusPin = D2;

void setup() {
  Serial.begin(115200);
  pinMode(armStatusPin, INPUT_PULLUP);
}

void loop() {

  int state = digitalRead(armStatusPin);

  Serial.print("D2 voltage state = ");
  Serial.println(state);

  if (state == LOW) {
    Serial.println("CIRCUIT COMPLETE -> ARMS DEPLOYED");
  } else {
    Serial.println("CIRCUIT OPEN -> ARMS NOT DEPLOYED");
  }

  delay(500);
}