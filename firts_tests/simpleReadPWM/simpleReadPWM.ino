int rc_channel1 = 9;
int rc_channel2 = 6;
int rc_channel3 = 5;
int rc_channel4 = 3;

void setup() {
  pinMode(rc_channel1, INPUT);
  pinMode(rc_channel2, INPUT);
  pinMode(rc_channel3, INPUT);
  pinMode(rc_channel4, INPUT);
  Serial.begin(9600);
}

void loop() {
  int raw_ch1 = pulseIn(rc_channel1, HIGH, 25000);
  int raw_ch2 = pulseIn(rc_channel2, HIGH, 25000);
  int raw_ch3 = pulseIn(rc_channel3, HIGH, 25000);
  int raw_ch4 = pulseIn(rc_channel4, HIGH, 25000);
  int ch1 = map(raw_ch1, 1000, 2000, -255, 255);
  int ch2 = map(raw_ch2, 1000, 2000, -255, 255);
  int ch3 = map(raw_ch3, 1000, 2000, -255, 255);
  int ch4 = map(raw_ch4, 1000, 2000, -255, 255);
  Serial.print("Ch1: ");
  Serial.print(ch1);
  Serial.print(", Ch2: ");
  Serial.print(ch2);
  Serial.print(", Ch3: ");
  Serial.print(ch3);
  Serial.print(", Ch4: ");
  Serial.println(ch4);
  delay(50);
}
