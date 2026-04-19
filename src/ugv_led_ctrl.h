void led_pin_init(){
  ledcSetup(2, FREQ, ANALOG_WRITE_BITS);  // Channel 2 for IO4
  ledcAttachPin(IO4_PIN, 2);
  ledcSetup(3, FREQ, ANALOG_WRITE_BITS);  // Channel 3 for IO5
  ledcAttachPin(IO5_PIN, 3);
}

void led_pwm_ctrl(int io4Input, int io5Input) {
  ledcWrite(2, constrain(io4Input, 0, 255));  // Channel 2 for IO4
  ledcWrite(3, constrain(io5Input, 0, 255));  // Channel 3 for IO5
}