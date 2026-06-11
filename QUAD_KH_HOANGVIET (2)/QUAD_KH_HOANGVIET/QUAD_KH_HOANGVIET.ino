#include <Arduino.h>
#include <stdio.h>
#include <SPI.h>
#include <driver/ledc.h>
#include "BMI160.h"
// DEFINE 
#define PWM_RECEIVER 
//#define SBUS_RECEIVER 
// HSPI
#define HSPI_SCLK          14
#define HSPI_MISO          12
#define HSPI_MOSI          13
#define HSPI_CS            27
// PWM Output (Motors)

#define MOTOR1_PIN         4
#define MOTOR2_PIN         5
#define MOTOR3_PIN         18
#define MOTOR4_PIN         19
#define LED_PIN            2

#if defined PWM_RECEIVER
// PWM Input
#define PWM_CH1           26
#define PWM_CH2           25
#define PWM_CH3           33
#define PWM_CH4           32
#define PWM_CH5           35
#define PWM_CH6           34
#endif
// VAR 
//TIMER
unsigned long t;
unsigned long print_time,blink_time,time_function_blink;
// PWM Input values
volatile uint32_t pwm_values[6] = {0};
//GYRO
uint8_t data[12];
int16_t acc[3], gyr[3];
float acc_mps[3], gyr_dps[3];
float gyr_fr_dps[3];
float angle_fr_d[3];
//FILTER
float acc_f[3] = {0},gyr_f[3] = {0};
float acc_f_prv[3] = {0},gyr_f_prv[3] = {0};
float acc_bias[3] = {0},gyr_bias[3] = {0};  
float B_accel = 0.014;    
float B_gyro = 0.485;
//RX
uint8_t arm_mode = 0;
unsigned long rx_time;
const float rx_default = 1500.0;
const float PWM_MIN = 800.0;
const float PWM_MAX = 2200.0;
float scale_rx[4],offset_rx[4];
float normalized_pwm[4];
float rx_desired[4] = {90.0,90.0,1000.0,90.0};
float rx_f[4];
float rx_f_prv[4];
float desired_value[3];
float Throttle;
//PID
const float dt_pid = 0.004 , freq_pid = 250.0;
float kp_inner[3];
float ki_inner[3];
float kd_inner[3];
float alpha_derivative_inner[3];
float pid_inner[3];

float error_inner[3];
float prev_error_inner[3];
float integral_inner[3];
float derivative_inner[3];
//MOTOR
const float ARM = 0.0f;
float throttle_m[4];
int esc[4];

#define MAX_PID_INNER 400.0
#define MAX_INTERGRAL_INNER 75.0
#define MAX_ANGLE_HOVER 60.0
#define MAX_ARM 1000.0
#define MIN_ARM 10.0
// SPI instance for HSPI
SPIClass hspi(HSPI);
// END VAR

// FUNCTION
void UART_CONFIG(void);
void SPI_CONFIG(void);
void PWM_CONFIG(bool calibrate_motor);
void RX_CONFIG(void);
void SENSOR_CONFIG(void);
void READ_GYRO();
void GYRO_CALIBRATE(bool cali_acc);
void MPC_Init(void);
void Caculate_MPC(void);

// ISR 
void IRAM_ATTR pwm_isr(uint8_t pin, uint8_t idx) {
    static uint32_t start_time[6] = {0};
    uint32_t now = micros();
    if (digitalRead(pin) == HIGH) {
        start_time[idx] = now;
    } else {
        pwm_values[idx] = now - start_time[idx];
    }
}
// END FUNCTION
void setup() {
  UART_CONFIG();
  SPI_CONFIG();
  RX_CONFIG();
  PWM_CONFIG(true);
  SENSOR_CONFIG();
  MPC_Init();
  Calibrate_Rx();
  GYRO_CALIBRATE(true);
  t = micros();
}
void loop() {
  READ_GYRO();
  READ_RX();
  Caculate_MPC();
  blink();
  DEBUG_DESIRED_VALUE();
  while(micros() - t < 4000);
  t = micros();
}
// FUNCTION HERE 
void UART_CONFIG(void) {
  Serial.begin(115200);
  delay(1000);
}
void SPI_CONFIG(void) {
  pinMode(HSPI_CS, OUTPUT);
  digitalWrite(HSPI_CS, HIGH);
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
}
auto pulse_to_duty = [](int pulse_width) -> uint32_t {
    return (pulse_width * 4095) / 2500;};
void PWM_CONFIG(bool calibrate_motor) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,1);
  ledcAttach(MOTOR1_PIN, 400, 12);
  ledcAttach(MOTOR2_PIN, 400, 12);
  ledcAttach(MOTOR3_PIN, 400, 12);
  ledcAttach(MOTOR4_PIN, 400, 12);
  if(calibrate_motor){
  Serial.println("Calibrate motor plug pin to CALIBRATE DONE && press any key");
  ledcWrite(MOTOR1_PIN, pulse_to_duty(2000));
  ledcWrite(MOTOR2_PIN, pulse_to_duty(2000));
  ledcWrite(MOTOR3_PIN, pulse_to_duty(2000));
  ledcWrite(MOTOR4_PIN, pulse_to_duty(2000));
  while(!Serial.available());
  }
  ledcWrite(MOTOR1_PIN, pulse_to_duty(1000));
  ledcWrite(MOTOR2_PIN, pulse_to_duty(1000));
  ledcWrite(MOTOR3_PIN, pulse_to_duty(1000));
  ledcWrite(MOTOR4_PIN, pulse_to_duty(1000));
  digitalWrite(LED_PIN,0);
  delay(100);
}

//END

//PID
void MPC_Init(void){
  //INNER
     float kp_rate_rp = 0.700993f;
     float kp_rate_y = 1.456010f;

     float ki_rate_rp = 1.265000f;  //0.199007f
     float ki_rate_y = 1.401011f;

     float kd_rate_rp = 0.112501f;
     float kd_rate_y = 0.138702f;

     float _alpha_derivative_inner = 0.012f;

  for(int i = 0 ; i < 2 ; i ++){
    //INNER
       kp_inner[i] = kp_rate_rp;
       ki_inner[i] = ki_rate_rp;
       kd_inner[i] = kd_rate_rp;
  }
       kp_inner[2] = kp_rate_y;
       ki_inner[2] = ki_rate_y;
       kd_inner[2] = kd_rate_y;
  //DERIVATIVE 
  for(int i = 0 ; i < 3 ; i ++){
    alpha_derivative_inner[i] = _alpha_derivative_inner;
  }
  print_pid_parameters();
}
void Caculate_MPC(void){
  if(arm_mode){
       MPC_RATE_MODE();
       MPC_CACULATE_THRUST();
  }
}
void MPC_RATE_MODE(void){
   for(int i = 0 ; i < 3 ; i ++){
    //ERROR
     error_inner[i] = desired_value[i] - gyr_fr_dps[i];
    //INTERGRAL
     integral_inner[i] += error_inner[i] * dt_pid;
     if(integral_inner[i] > MAX_INTERGRAL_INNER) integral_inner[i] = MAX_INTERGRAL_INNER;
     if(integral_inner[i] < -MAX_INTERGRAL_INNER) integral_inner[i] = -MAX_INTERGRAL_INNER;
     //DERIVATIVE
     derivative_inner[i] = (1.0 - alpha_derivative_inner[i]) * derivative_inner[i] + alpha_derivative_inner[i] * (error_inner[i] - prev_error_inner[i]) * freq_pid;
    //Caculate PID
    pid_inner[i] = kp_inner[i] * error_inner[i] + ki_inner[i] * integral_inner[i] + kd_inner[i] * derivative_inner[i];
    if(pid_inner[i] > MAX_PID_INNER) pid_inner[i] = MAX_PID_INNER;
    if(pid_inner[i] < -MAX_PID_INNER) pid_inner[i] = -MAX_PID_INNER;
    //update error
    prev_error_inner[i] = error_inner[i];
   }
}
void MPC_RATE_MODE_RESET(void) {
    for (int i = 0; i < 3; i++) {
        error_inner[i] = 0.0f;
        integral_inner[i] = 0.0f;
        derivative_inner[i] = 0.0f;
        prev_error_inner[i] = 0.0f;
        pid_inner[i] = 0.0f;
    }
}
void MPC_CACULATE_THRUST(void){
    throttle_m[0] = Throttle - pid_inner[0] - pid_inner[1] - pid_inner[2]; //M1
    throttle_m[1] = Throttle - pid_inner[0] + pid_inner[1] + pid_inner[2]; //M2
    throttle_m[2] = Throttle + pid_inner[0] + pid_inner[1] - pid_inner[2]; //M3
    throttle_m[3] = Throttle + pid_inner[0] - pid_inner[1] + pid_inner[2]; //M4
  for(int i = 0 ; i < 4 ; i ++){
    if(throttle_m[i] > MAX_ARM) throttle_m[i] = MAX_ARM;
    if(throttle_m[i] < MIN_ARM) throttle_m[i] = MIN_ARM;
    esc[i] = 1000 + (int) ((throttle_m[i] / MAX_ARM) * 1000);
  }
  CONTROL_ESC();
}
void CONTROL_ESC(void){
  ledcWrite(MOTOR1_PIN, pulse_to_duty(esc[0]));
  ledcWrite(MOTOR2_PIN, pulse_to_duty(esc[1]));
  ledcWrite(MOTOR3_PIN, pulse_to_duty(esc[2]));
  ledcWrite(MOTOR4_PIN, pulse_to_duty(esc[3]));
}
void STOP_MOTOR_FAST(void){
  ledcWrite(MOTOR1_PIN, pulse_to_duty(1000));
  ledcWrite(MOTOR2_PIN, pulse_to_duty(1000));
  ledcWrite(MOTOR3_PIN, pulse_to_duty(1000));
  ledcWrite(MOTOR4_PIN, pulse_to_duty(1000));
  for(int i = 0 ; i < 4 ; i ++){
    esc[i] = 1000;
    throttle_m[i] = 0.0;
  }
}
