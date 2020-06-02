#include <Wire.h> //Include Wire for communication with gyro
#include <EEPROM.h>

//Declaring global variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36], start, data;
boolean new_function_request, first_angle;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int receiver_input[5];
int loop_counter, gyro_address, vibration_counter;
int temperature;
long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector, vibration_total_result;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int acc_axis[4], gyro_axis[4];
double gyro_pitch, gyro_roll, gyro_yaw;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int cal_int;
double gyro_axis_cal[4];

void setup()
{
  Serial.begin(57600); // Start the serial port
  Wire.begin();        // Start the wire library as master
  TWBR = 12;           // Set the I2C clock speed to 400kHz
  gyro_address = 0x68; // I2C address of the MPU-6050

  // Arduino UNO pins default to inputs, so they need
  // to be explicitly declared as inputs.
  DDRD |= B11110000; // Configure digital port 4, 5, 6 and 7 as output
  DDRB |= B00010000; // Configure digital port 12 as output

  PCICR |= (1 << PCIE0);   // Set PCIE0 to enable PCMK0 scan
  PCMSK0 |= (1 << PCINT0); // Set PCINT0 (diigital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1); // Set PCINT1 (diigital input 9) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2); // Set PCINT2 (diigital input 10) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3); // Set PCINT3 (diigital input 11) to trigger an interrupt on state change

  for (data = 0; data <= 35; data++)
    eeprom_data[data] = EEPROM.read(data);

  Serial.println("Setting Up Gyro");
  set_gyro_registers(); // Set the specific gyro registers

  // while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')
  // {
  //   delay(500);
  //   digitalWrite(12, !digitalRead(12));
  // }

  Serial.println("Setting Up RC Controll");
  wait_for_receiver(); // Read RC Controll signal

  while (Serial.available())
    data = Serial.read();
}

void loop()
{
  while (zero_timer + 4000 > micros())
    zero_timer = micros();

  if (Serial.available() > 0)
  {
    data = Serial.read();
    delay(100);
    while (Serial.available() > 0)
      loop_counter = Serial.read();
    new_function_request = true;
    loop_counter = 0;
    cal_int = 0;
    start = 0;
    first_angle = false;
    //Confirm the choice on the serial monitor.
    if (data == 'r')
      Serial.println("Reading receiver signals.");
    if (data == 'a')
      Serial.println("Print the quadcopter angles.");
    if (data == 'a')
      Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter).");
    if (data == '1')
      Serial.println("Test motor 1 (right front CCW.)");
    if (data == '2')
      Serial.println("Test motor 2 (right rear CW.)");
    if (data == '3')
      Serial.println("Test motor 3 (left rear CCW.)");
    if (data == '4')
      Serial.println("Test motor 4 (left front CW.)");
    if (data == '5')
      Serial.println("Test all motors together");

    //Let's create a small delay so the message stays visible for 2.5 seconds.
    //We don't want the ESC's to beep and have to send a 1000us pulse to the ESC's.
    for (vibration_counter = 0; vibration_counter < 625; vibration_counter++)
    {                     //Do this loop 625 times
      delay(3);           //Wait 3000us.
      esc_1 = 1000;       //Set the pulse for ESC 1 to 1000us.
      esc_2 = 1000;       //Set the pulse for ESC 1 to 1000us.
      esc_3 = 1000;       //Set the pulse for ESC 1 to 1000us.
      esc_4 = 1000;       //Set the pulse for ESC 1 to 1000us.
      esc_pulse_output(); //Send the ESC control pulses.
    }
    vibration_counter = 0;
  }

  receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  if (receiver_input_channel_3 < 1025)
    new_function_request = false; //If the throttle is in the lowest position set the request flag to false.

  ////////////////////////////////////////////////////////////////////////////////////////////
  //Run the ESC calibration program to start with.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if (data == 0 && new_function_request == false)
  { //Only start the calibration mode at first start.
    Serial.println("ESC calibration program");
    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    esc_1 = receiver_input_channel_3;                       //Set the pulse for motor 1 equal to the throttle channel.
    esc_2 = receiver_input_channel_3;                       //Set the pulse for motor 2 equal to the throttle channel.
    esc_3 = receiver_input_channel_3;                       //Set the pulse for motor 3 equal to the throttle channel.
    esc_4 = receiver_input_channel_3;                       //Set the pulse for motor 4 equal to the throttle channel.
    esc_pulse_output();                                     //Send the ESC control pulses.
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'r' print the receiver signals.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if (data == 'r')
  {
    loop_counter++;                                         //Increase the loop_counter variable.
    receiver_input_channel_1 = convert_receiver_channel(1); //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2); //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

    if (loop_counter == 125)
    {                   //Print the receiver values when the loop_counter variable equals 250.
      print_signals();  //Print the receiver values on the serial monitor.
      loop_counter = 0; //Reset the loop_counter variable.
    }

    //For starting the motors: throttle low and yaw left (step 1).
    if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)
      start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if (start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450)
      start = 2;
    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)
      start = 0;

    esc_1 = 1000;       //Set the pulse for ESC 1 to 1000us.
    esc_2 = 1000;       //Set the pulse for ESC 1 to 1000us.
    esc_3 = 1000;       //Set the pulse for ESC 1 to 1000us.
    esc_4 = 1000;       //Set the pulse for ESC 1 to 1000us.
    esc_pulse_output(); //Send the ESC control pulses.
  }
}

void esc_pulse_output()
{
  zero_timer = micros();
  PORTD |= B11110000;                   //Set port 4, 5, 6 and 7 high at once
  timer_channel_1 = esc_1 + zero_timer; //Calculate the time when digital port 4 is set low.
  timer_channel_2 = esc_2 + zero_timer; //Calculate the time when digital port 5 is set low.
  timer_channel_3 = esc_3 + zero_timer; //Calculate the time when digital port 6 is set low.
  timer_channel_4 = esc_4 + zero_timer; //Calculate the time when digital port 7 is set low.

  while (PORTD >= 16)
  {                            //Execute the loop until digital port 4 to 7 is low.
    esc_loop_timer = micros(); //Check the current time.
    if (timer_channel_1 <= esc_loop_timer)
      PORTD &= B11101111; //When the delay time is expired, digital port 4 is set low.
    if (timer_channel_2 <= esc_loop_timer)
      PORTD &= B11011111; //When the delay time is expired, digital port 5 is set low.
    if (timer_channel_3 <= esc_loop_timer)
      PORTD &= B10111111; //When the delay time is expired, digital port 6 is set low.
    if (timer_channel_4 <= esc_loop_timer)
      PORTD &= B01111111; //When the delay time is expired, digital port 7 is set low.
  }
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function)
{
  // First we declare some local variables
  byte channel, reverse;
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111; // What channel corresponds with the specific function
  if (eeprom_data[function + 23] & 0b10000000)
    reverse = 1; // Reverse channel when most significant bit is set
  else
    reverse = 0; // If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            // Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  // Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; // Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   // Store the high value for the specific receiver input channel

  if (actual < center)
  { // The actual receiver value is lower than the center value
    if (actual < low)
      actual = low;                                                      // Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low); // Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)
      return 1500 - difference; // If the channel is reversed
    else
      return 1500 + difference; // If the channel is not reversed
  }
  else if (actual > center)
  { //The actual receiver value is higher than the center value
    if (actual > high)
      actual = high;                                                      //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center); //Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)
      return 1500 - difference; //If the channel is reversed
    else
      return 1500 + difference; //If the channel is not reversed
  }
  else
    return 1500;
}

void print_signals()
{
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("  Roll:");
  if (receiver_input_channel_1 - 1480 < 0)
    Serial.print("<<<");
  else if (receiver_input_channel_1 - 1520 > 0)
    Serial.print(">>>");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Pitch:");
  if (receiver_input_channel_2 - 1480 < 0)
    Serial.print("^^^");
  else if (receiver_input_channel_2 - 1520 > 0)
    Serial.print("vvv");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Throttle:");
  if (receiver_input_channel_3 - 1480 < 0)
    Serial.print("vvv");
  else if (receiver_input_channel_3 - 1520 > 0)
    Serial.print("^^^");
  else
    Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if (receiver_input_channel_4 - 1480 < 0)
    Serial.print("<<<");
  else if (receiver_input_channel_4 - 1520 > 0)
    Serial.print(">>>");
  else
    Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}

void set_gyro_registers()
{
  Wire.beginTransmission(gyro_address); // Start communication
  Wire.write(0x6B);                     // We want to write to the PWR_MGMT_1 register
  Wire.write(0x00);                     // Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();               // End transmission with the gyro

  Wire.beginTransmission(gyro_address); // Start communication
  Wire.write(0x1B);                     // We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                     // Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();               // End transmission with the gyro

  Wire.beginTransmission(gyro_address); // Start communication
  Wire.write(0x1C);                     // We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                     // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();               // End transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search
  Wire.write(0x1B);                     //Start reading @ register 0x1B
  Wire.endTransmission();               //End the transmission
  Wire.requestFrom(gyro_address, 1);    //Request 1 bytes from the gyro
  while (Wire.available() < 1)
    ; //Wait until the 6 bytes are received
  if (Wire.read() != 0x08)
  {                         //Check if the value is 0x08
    digitalWrite(12, HIGH); //Turn on the warning led
    while (1)
      delay(10); //Stay in this loop for ever
  }

  Wire.beginTransmission(gyro_address); //Start communication with the address found during search
  Wire.write(0x1A);                     //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                     //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();               //End the transmission with the gyro

  Serial.println("Gyro SetUp Success!!");
}

//Checck if the receiver values are valid within 10 seconds
void wait_for_receiver()
{
  byte zero = 0; //Set all bits in the variable zero to 0
  while (zero < 15)
  { //Stay in this loop until the 4 lowest bits are set
    if (receiver_input[1] < 2100 && receiver_input[1] > 900)
      zero |= 0b00000001; //Set bit 0 if the receiver pulse 1 is within the 900 - 2100 range
    if (receiver_input[2] < 2100 && receiver_input[2] > 900)
      zero |= 0b00000010; //Set bit 1 if the receiver pulse 2 is within the 900 - 2100 range
    if (receiver_input[3] < 2100 && receiver_input[3] > 900)
      zero |= 0b00000100; //Set bit 2 if the receiver pulse 3 is within the 900 - 2100 range
    if (receiver_input[4] < 2100 && receiver_input[4] > 900)
      zero |= 0b00001000; //Set bit 3 if the receiver pulse 4 is within the 900 - 2100 range
    delay(500);           //Wait 500 milliseconds
    Serial.println(receiver_input[1]);
    Serial.println(receiver_input[2]);
    Serial.println(receiver_input[3]);
    Serial.println(receiver_input[4]);
    Serial.println("RC Controll SetUp Success!!");
  }
}

//This routine is called every time input 8, 9, 10 or 11 changed state.
ISR(PCINT0_vect)
{
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001)
  { //Is input 8 high?
    if (last_channel_1 == 0)
    {                         //Input 8 changed from 0 to 1.
      last_channel_1 = 1;     //Remember current input state.
      timer_1 = current_time; //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1)
  {                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                         //Remember current input state.
    receiver_input[1] = current_time - timer_1; //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if (PINB & B00000010)
  { //Is input 9 high?
    if (last_channel_2 == 0)
    {                         //Input 9 changed from 0 to 1.
      last_channel_2 = 1;     //Remember current input state.
      timer_2 = current_time; //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1)
  {                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                         //Remember current input state.
    receiver_input[2] = current_time - timer_2; //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if (PINB & B00000100)
  { //Is input 10 high?
    if (last_channel_3 == 0)
    {                         //Input 10 changed from 0 to 1.
      last_channel_3 = 1;     //Remember current input state.
      timer_3 = current_time; //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1)
  {                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                         //Remember current input state.
    receiver_input[3] = current_time - timer_3; //Channel 3 is current_time - timer_3.
  }
  //Channel 4=========================================
  if (PINB & B00001000)
  { //Is input 11 high?
    if (last_channel_4 == 0)
    {                         //Input 11 changed from 0 to 1.
      last_channel_4 = 1;     //Remember current input state.
      timer_4 = current_time; //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1)
  {                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                         //Remember current input state.
    receiver_input[4] = current_time - timer_4; //Channel 4 is current_time - timer_4.
  }
}