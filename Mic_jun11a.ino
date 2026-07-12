const int THRESHOLD = 350;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int micValue = analogRead(A0);

  Serial.print("Mic: ");
  Serial.print(micValue);

  if (micValue > THRESHOLD) {
    Serial.println("  --> Sound Detected");
  } else {
    Serial.println();
  }

  delay(100);
}