/*
 * PRINT OUT ANALOG INPUT ON A0 TO LED 9
 * EXAMPLE WITH PULSE WIDTH MODULATION,
 * ANALOG SIGNAL READING AND SERIAL MONITORING.
 */

int ledPin = 9;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(A0);
  float fadeVoltage = sensorValue * (255.0 / 1023.0);
  Serial.println(fadeVoltage);
  analogWrite(ledPin, fadeVoltage);
  delay(30);
}
