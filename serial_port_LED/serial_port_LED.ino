const int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char ledPinState = Serial.read();
    if (ledPinState == '1') {
      digitalWrite(ledPin, HIGH);
    }
    if (ledPinState == '0') {
      digitalWrite(ledPin, LOW);
    }
  }
}
