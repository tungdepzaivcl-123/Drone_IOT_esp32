

void RX_CONFIG(void) {
  #if defined PWM_RECEIVER
  pinMode(PWM_CH1, INPUT);
  pinMode(PWM_CH2, INPUT);
  pinMode(PWM_CH3, INPUT);
  pinMode(PWM_CH4, INPUT);
  pinMode(PWM_CH5, INPUT);
  pinMode(PWM_CH6, INPUT);
  attachInterrupt(PWM_CH1, [](){ pwm_isr(PWM_CH1, 0); }, CHANGE);
  attachInterrupt(PWM_CH2, [](){ pwm_isr(PWM_CH2, 1); }, CHANGE);
  attachInterrupt(PWM_CH3, [](){ pwm_isr(PWM_CH3, 2); }, CHANGE);
  attachInterrupt(PWM_CH4, [](){ pwm_isr(PWM_CH4, 3); }, CHANGE);
  attachInterrupt(PWM_CH5, [](){ pwm_isr(PWM_CH5, 4); }, CHANGE);
  attachInterrupt(PWM_CH6, [](){ pwm_isr(PWM_CH6, 5); }, CHANGE);
 
  #endif
  #if defined SBUS_RECEIVER

  #endif

}
void Calibrate_Rx(){
  float pwm_values_max[4],pwm_values_min[4];
  pwm_values_max[0] = 2004;
  pwm_values_max[1] = 2003;
  pwm_values_max[2] = 2002;
  pwm_values_max[3] = 2001;
  pwm_values_min[0] = 990;
  pwm_values_min[1] = 991;
  pwm_values_min[2] = 989;
  pwm_values_min[3] = 988;
  for (int i = 0; i < 4; i++) {
    float range = pwm_values_max[i] - pwm_values_min[i];
    scale_rx[i] = 2.0 / range;
    offset_rx[i] = -1.0 * (pwm_values_max[i] + pwm_values_min[i]) / range;
  }
  Serial.print("scale_rx: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(scale_rx[i], 6);
    if (i < 3) Serial.print(", ");
  }
  Serial.println();
  Serial.print("offset_rx: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(offset_rx[i], 6);
    if (i < 3) Serial.print(", ");
  }
  Serial.println();
}
void READ_RX(){
   for(int i = 0 ; i < 4 ; i ++){
     if(pwm_values[i] < PWM_MIN) pwm_values[i] = PWM_MIN;
     if(pwm_values[i] > PWM_MAX) pwm_values[i] = PWM_MAX;
     normalized_pwm[i] = pwm_values[i] * scale_rx[i] + offset_rx[i];
     rx_f[i] = 0.9f * rx_f_prv[i] + 0.1f * normalized_pwm[i];
     rx_f_prv[i] = rx_f[i];
     if(i < 2){
      desired_value[i] =(int)(normalized_pwm[i] * rx_desired[i]);
     }
     if(i > 2 && i < 4){
      desired_value[i - 1] =(int)(normalized_pwm[i] * rx_desired[i]);
     }
     if(i == 2){
      Throttle =(int) ((normalized_pwm[i] * rx_desired[i] + 1000.0) * 0.5);
     }
   }
   if(pwm_values[4] > 1400){
    arm_mode = 1;
   }
   else{
    arm_mode = 0;
    STOP_MOTOR_FAST();
    MPC_RATE_MODE_RESET();
   }
}