/*
   First FLYSKY RC controller test

   Connections:

   FS-A6 Receiver

   BAT (+) => 5v pin
   BAT (-) => GND pin
   CH (1 - 6) => Pin ~3 (Change connection between pins to test all channels)
*/

void setup() {
  pinMode(3, INPUT);
  Serial.begin(9600);
}

void loop() {
  int v = pulseIn(3, HIGH, 25000);
  Serial.println(v);
  delay(500);
}
