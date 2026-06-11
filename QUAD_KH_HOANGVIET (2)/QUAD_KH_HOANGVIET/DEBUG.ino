//DEBUG
void DEBUG_RX(void) {
  if (t - print_time >= 100000) {
    print_time = t;
    Serial.printf("CH1:%4d CH2:%4d CH3:%4d CH4:%4d CH5:%4d CH6:%4d\n",pwm_values[0], pwm_values[1], pwm_values[2], pwm_values[3],pwm_values[4], pwm_values[5]);
  }
}
void DEBUG_DESIRED_VALUE(void) {
  if (t - print_time >= 100000) {
    print_time = t;
    Serial.printf("R:%.4f P:%.4f Thr:%.4f Y:%.4f\n",desired_value[0],desired_value[1],Throttle,desired_value[2]);
  }
}
void print_pid_parameters(void) {
    printf("PID Parameters:\n");
    printf("--------------------------------------------------\n");
    // In các hệ số cho vòng trong (inner loop)
    printf("Inner Loop (Rate_Roll,Rate_Pitch,Rate_Yaw):\n");
    printf("Kp_inner: %.6f, %.6f, %.6f\n", kp_inner[0], kp_inner[1], kp_inner[2]);
    printf("Ki_inner: %.6f, %.6f, %.6f\n", ki_inner[0], ki_inner[1], ki_inner[2]);
    printf("Kd_inner: %.6f, %.6f, %.6f\n", kd_inner[0], kd_inner[1], kd_inner[2]);
    printf("Alpha_derivative_inner: %.6f, %.6f, %.6f\n", alpha_derivative_inner[0], alpha_derivative_inner[1], alpha_derivative_inner[2]);
    printf("--------------------------------------------------\n");
}
void blink(void){
  if(arm_mode){
    time_function_blink = 100000;
  }
  else{
    time_function_blink = 1000000;
  }
  if(t - blink_time > time_function_blink){
    blink_time = t;
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));
  }
}