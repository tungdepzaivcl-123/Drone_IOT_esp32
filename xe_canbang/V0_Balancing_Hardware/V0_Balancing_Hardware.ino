#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <esp_wifi.h>

// ================= 1. CẤU TRÚC DỮ LIỆU (KHÔNG ĐỔI) =================
typedef struct __attribute__((packed)) {
  int lx, ly, rx, ry;
  bool lock;
  int throttle;
} struct_command;

typedef struct __attribute__((packed)) {
  bool mpu, baro, mag;
  int16_t pitch_val, roll_val;
  float altitude;
  bool isReady;
  int m1, m2, m3, m4;
  int heading;
} struct_status;

struct_command cmd;
struct_status fb;
MPU6050 mpu(Wire);

// ===== CẤU HÌNH PHẦN CỨNG =====
const int motorPins[] = {27, 26, 14, 13}; // FR, FL, RR, RL
const int PWM_MIN = 1024, PWM_MAX = 2048, MIN_IDLE = 1150;
const int THR_LIMIT = 1850; 
const unsigned long LOOP_TIME_US = 4000; // 250Hz

uint8_t remoteMAC[] = {0xD4, 0xE9, 0xF4, 0xE8, 0x30, 0xC8}; 

// PID & FILTER (Đã tinh chỉnh cho bay ổn định)
float Kp = 1.25, Ki = 0.05, Kd = 0.45;
float fP = 0, fR = 0;
float iPitch = 0, iRoll = 0, prevEP = 0, prevER = 0;
float pPID, rPID, yPID;

// TRANG THÁI & AN TOÀN
bool isArmed = false;
unsigned long lastRecvTime = 0, lastLoopTime = 0;

// ================= 2. HÀM XỬ LÝ DỮ LIỆU =================

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(cmd)) { 
    memcpy(&cmd, data, sizeof(cmd)); 
    lastRecvTime = millis(); 
  }
}

void readSensors() {
  mpu.update();
  // Bộ lọc Complementary Filter giúp góc mượt hơn
  fP = (fP * 0.96) + (mpu.getAngleX() * 0.04);
  fR = (fR * 0.96) + (mpu.getAngleY() * 0.04);
  fb.pitch_val = (int16_t)fP; 
  fb.roll_val = (int16_t)fR;
}

void safetyCheck() {
  unsigned long now = millis();
  // 1. Mất kết nối > 200ms => Ngắt động cơ ngay lập tức
  if (now - lastRecvTime > 200) {
    isArmed = false;
    return;
  } 
  
  // 2. Logic Arming: Gạt khóa OFF + Hạ ga về Min mới cho phép Arm
  if (cmd.lock) {
    isArmed = false;
    iPitch = 0; iRoll = 0;
  } else if (!isArmed && cmd.throttle < 1050) {
    isArmed = true; 
  }

  // 3. Tự ngắt nếu nghiêng quá 55 độ (Tránh lật nhào)
  if (abs(fP) > 55 || abs(fR) > 55) isArmed = false;
  
  fb.isReady = isArmed;
}

void calculatePID(float dt) {
  if (!isArmed) {
    pPID = 0; rPID = 0; yPID = 0; iPitch = 0; iRoll = 0;
    return;
  }

  // Chuyển đổi stick từ Remote sang góc mục tiêu (+- 20 độ)
  float targetP = (abs(cmd.ry) > 10) ? (cmd.ry - 1500) * 0.04 : 0;
  float targetR = (abs(cmd.rx) > 10) ? (cmd.rx - 1500) * 0.04 : 0;

  float eP = targetP - fP;
  float eR = targetR - fR;

  // I-term: Chỉ tích lũy khi ga đủ cao để tránh "vọt" khi chưa cất cánh
  if (cmd.throttle > 1100) {
    iPitch = constrain(iPitch + eP * dt, -100, 100);
    iRoll  = constrain(iRoll  + eR * dt, -100, 100);
  }

  // Công thức PID: $$Output = K_p \cdot e + K_i \cdot \int e \,dt + K_d \cdot \frac{de}{dt}$$
  float dP = (eP - prevEP) / dt;
  float dR = (eR - prevER) / dt;

  pPID = (Kp * eP) + (Ki * iPitch) + (Kd * dP);
  rPID = (Kp * eR) + (Ki * iRoll)  + (Kd * dR);
  yPID = (abs(cmd.lx - 1500) > 20) ? (cmd.lx - 1500) * 0.5 : 0; // Đơn giản hóa Yaw

  prevEP = eP; prevER = eR;
}

void applyMixer() {
  int thr = map(cmd.throttle, 1000, 2000, PWM_MIN, THR_LIMIT);

  if (!isArmed) {
    for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], PWM_MIN);
    fb.m1 = fb.m2 = fb.m3 = fb.m4 = PWM_MIN;
  } else {
    // Mixer chuẩn X-Config
    fb.m1 = constrain(thr - pPID - rPID - yPID, MIN_IDLE, PWM_MAX); // Front Right
    fb.m2 = constrain(thr - pPID + rPID + yPID, MIN_IDLE, PWM_MAX); // Front Left
    fb.m3 = constrain(thr + pPID - rPID + yPID, MIN_IDLE, PWM_MAX); // Rear Right
    fb.m4 = constrain(thr + pPID + rPID - yPID, MIN_IDLE, PWM_MAX); // Rear Left

    ledcWrite(motorPins[0], fb.m1);
    ledcWrite(motorPins[1], fb.m2);
    ledcWrite(motorPins[2], fb.m3);
    ledcWrite(motorPins[3], fb.m4);
  }
}

// ================= 3. KHỞI TẠO & VÒNG LẶP =================

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Setup PWM 12-bit
  for (int i = 0; i < 4; i++) {
    ledcAttach(motorPins[i], 250, 12); 
    ledcWrite(motorPins[i], PWM_MIN);
  }

  if (mpu.begin() == 0) {
    delay(1000);
    mpu.calcOffsets(); // ĐỂ MÁY BAY NẰM YÊN KHI CẮM PIN
    fb.mpu = true;
  }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, remoteMAC, 6);
    peer.channel = 6;
    esp_now_add_peer(&peer);
  }
}

void loop() {
  unsigned long now = micros();
  if (now - lastLoopTime >= LOOP_TIME_US) {
    float dt = (now - lastLoopTime) / 1000000.0;
    lastLoopTime = now;

    readSensors();
    safetyCheck();
    calculatePID(dt);
    applyMixer();

    // Gửi Feedback về Remote mỗi 100ms
    static unsigned long lastS = 0;
    if (millis() - lastS > 100) {
      esp_now_send(remoteMAC, (uint8_t *)&fb, sizeof(fb));
      lastS = millis();
    }
  }
}