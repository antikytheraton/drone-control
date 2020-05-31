int in1 = 12;
int in2 = 13;

// the following are all ~PWM capable ports
int enable1 = 11;
int rc_channel4 = 3;
int switchD = 5;

void setup()
{
  pinMode(rc_channel4, INPUT);
  pinMode(switchD, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enable1, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  int pwm = 0;
  int swd = pulseIn(switchD, HIGH, 25000);
  int rc4 = pulseIn(rc_channel4, HIGH, 25000);

  Serial.print("switch: ");
  Serial.print(swd);
  Serial.print(" raw channel4: ");
  Serial.print(rc4);

  if (rc4 == 0)
  {
    Serial.println(" no signal");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enable1, 0);
  }
  else if (rc4 > 1530)
  {
    pwm = map(rc4, 1530, 2000, 0, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enable1, pwm);
    Serial.print(" right stick speed: ");
    Serial.println(pwm);
  }
  else if (rc4 < 1460)
  {
    pwm = map(rc4, 1460, 1000, 0, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enable1, pwm);
    Serial.print(" left stick speed: ");
    Serial.println(pwm);
  }
  else
  {
    Serial.println(" stick centered");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enable1, 0);
  }
  delay(10);
}
