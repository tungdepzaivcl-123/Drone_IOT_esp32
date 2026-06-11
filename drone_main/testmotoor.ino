#include <Wire.h>
#include <MPU6050_light.h>

// PINOUT: FR-M1(27), FL-M2(26), RR-M3(14), RL-M4(13)
const int motorPins[] = {27, 26, 14, 13}; 
const int freq = 250, res = 12;
const int PWM_OFF = 1024;
const int PWM_ON = 1350; 

MPU6050 mpu(Wire);

// --- BIẾN LỌC NHIỄU PHẦN MỀM (EMA Filter) ---
float alpha = 0.2;  // Càng nhỏ càng mượt nhưng trễ. 0.2 là mức cân bằng.
float smoothX = 0, smoothY = 0;

void setupMPUHardwareFilter() {
  Wire.beginTransmission(0x68); 
  Wire.write(0x1A);             
  Wire.write(0x03);             // DLPF ~42Hz
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); 

  for (int i = 0; i < 4; i++) {
    ledcAttach(motorPins[i], freq, res);
    ledcWrite(motorPins[i], PWM_OFF);
  }

  if (mpu.begin() == 0) {
    setupMPUHardwareFilter(); 
    delay(1000);
    mpu.calcOffsets();
    Serial.println("--- DA SUA LOGIC MOTOR: NGHIENG TRAI NANG TRAI ---");
  } else {
    while(1);
  }
}

void loop() {
  mpu.update();

  // 1. LỌC NHIỄU EMA
  smoothX = (alpha * mpu.getAngleX()) + (1.0 - alpha) * smoothX;
  smoothY = (alpha * mpu.getAngleY()) + (1.0 - alpha) * smoothY;

  int m1 = PWM_OFF, m2 = PWM_OFF, m3 = PWM_OFF, m4 = PWM_OFF;

  // 2. LOGIC KIỂM TRA TRỤC

  // --- KIỂM TRA PITCH (TRƯỚC/SAU) ---
  if (smoothX > 10) { 
    Serial.print("NGHIENG TRUOC -> Nang TRUOC (M1, M2) | ");
    m1 = PWM_ON; m2 = PWM_ON; 
  } 
  else if (smoothX < -10) { 
    Serial.print("NGHIENG SAU -> Nang SAU (M3, M4) | ");
    m3 = PWM_ON; m4 = PWM_ON;
  }

  // --- KIỂM TRA ROLL (TRAI/PHAI) - ĐÃ FIX LOGIC ---
  if (smoothY > 10) { 
    // Khi nghiêng TRÁI, motor bên TRÁI (M2+M4) phải gồng lên để đẩy drone về bên phải
    Serial.print("NGHIENG TRAI -> Nang motor ben TRAI (M2+M4) | ");
    m2 = PWM_ON; m4 = PWM_ON; // M2(FL) + M4(RL) - motor bên TRÁI
  }
  else if (smoothY < -10) { 
    // Khi nghiêng PHẢI, motor bên PHẢI (M1+M3) phải gồng lên
    Serial.print("NGHIENG PHAI -> Nang motor ben PHAI (M1+M3) | ");
    m1 = PWM_ON; m3 = PWM_ON; // M1(FR) + M3(RR) - motor bên PHẢI
  }

  // 3. XUẤT LỆNH RA MOTOR
  ledcWrite(motorPins[0], m1); // FR (Trước Phải)
  ledcWrite(motorPins[1], m2); // FL (Trước Trái)
  ledcWrite(motorPins[2], m3); // RR (Sau Phải)
  ledcWrite(motorPins[3], m4); // RL (Sau Trái)

  if (abs(smoothX) > 10 || abs(smoothY) > 10) {
    Serial.printf("X: %.2f | Y: %.2f\n", smoothX, smoothY);
  }

  delay(10); 
}