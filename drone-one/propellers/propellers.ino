int esc_1, esc_2, esc_3, esc_4;
unsigned long zero_timer;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;

void setup()
{
    DDRD |= B11110000; // Set PORTD (4,5,6,7) as output
    DDRB |= B00010000; // Set PORTB (12) as output
}

void loop()
{
    esc_1 = 0;
    esc_2 = 1000;
    esc_3 = 0;
    esc_4 = 0;
    esc_pulse_output();
}

void esc_pulse_output()
{
    zero_timer = micros();
    PORTD |= B11110000;
    timer_channel_1 = esc_1 + zero_timer;
    timer_channel_2 = esc_2 + zero_timer;
    timer_channel_3 = esc_3 + zero_timer;
    timer_channel_4 = esc_4 + zero_timer;

    while (PORTD >= 16)
    {
        esc_loop_timer = micros();
        if (timer_channel_1 <= esc_loop_timer)
            PORTD &= B11101111;
        if (timer_channel_2 <= esc_loop_timer)
            PORTD &= B11011111;
        if (timer_channel_3 <= esc_loop_timer)
            PORTD &= B10111111;
        if (timer_channel_4 <= esc_loop_timer)
            PORTD &= B01111111;
    }
}