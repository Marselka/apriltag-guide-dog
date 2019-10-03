void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    if (b == 'v') {
      Serial.write("Ok.");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}
