/*
 * ============================================================
 *  DRONE FIRMWARE V5.0 — FULL BUG FIX RELEASE
 *
 *  [V5.0 — Fixes tổng hợp từ phân tích V4.6]
 *
 *  🔴 CRITICAL FIXES:
 *  #1 Race condition cmd struct (Core0/Core1):
 *     → Double buffer + portMUX spinlock
 *     → OnDataRecv ghi vào cmd_buf (Core0)
 *     → loop() copy vào cmd dưới critical section (Core1)
 *
 *  #2 smooth_m[] không reset khi disarm:
 *     → Reset về MIN_IDLE mỗi khi !isArmed
 *     → Tránh motor chạy cao khi rearm
 *
 *  🟡 LOGIC FIXES:
 *  #3 D-term: derivative of error → D on measurement:
 *     → raw_d = -(smoothGyro[i] - prev_gyro[i]) / dt
 *     → Loại bỏ derivative kick khi stick thay đổi đột ngột
 *
 *  #4 Anti-windup decay 0.995f → 0.9998f:
 *     → 0.995^250 = 28% còn lại sau 1s (I-term vô dụng!)
 *     → 0.9998^250 = 95% còn lại sau 1s (hợp lý)
 *
 *  #5 Bỏ throttle-dependent gain scaling (thr_scale):
 *     → Gain phi tuyến theo ga → khó tune, điểm làm việc dịch
 *     → Thay bằng gains cố định (đã giảm xuống mức an toàn)
 *
 *  🔵 PERFORMANCE FIXES:
 *  #6 I2C 100kHz → 400kHz:
 *     → GY-87 hỗ trợ 400kHz fast-mode
 *     → Giải phóng ~16% CPU budget từ 22% → 6% per loop
 *
 *  #7 Wire.setTimeOut(3ms):
 *     → Tránh mpu.update() block vô hạn khi I2C hang
 *     → Ngăn ESP32 watchdog reboot giữa flight
 *
 *  [GIỮ NGUYÊN từ V4.6]
 *  - Motor pins: FR-M1(27), FL-M2(26), RR-M3(14), RL-M4(13)
 *  - MIN_IDLE = 260, HOVER_THR = 620
 *  - BMP280 Alt-Hold (dead zone 1460-1540)
 *  - ESP-NOW CH1, Web Config UI, NVS PID save
 *  - Tilt failsafe 45° / 35°+200dps
 *  - Soft ramp 600ms sau ARM
 *  - Sign Paradox Mixer fix (V4.3)
 *
 *  [Convention — GROUND TRUTH từ testmotoor.ino]
 *  - AngleX > 0 = nghiêng TRƯỚC → nâng M1(FR)+M2(FL)
 *  - AngleY > 0 = nghiêng TRÁI  → nâng M2(FL)+M4(RL)
 *
 *  I2C: SDA=21, SCL=22 | Board: GY-87 (MPU6050+BMP280)
 *  RC:  ESP-NOW CH1, 50Hz | Struct khớp taycamxin.ino
 * ============================================================
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_BMP280.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>

// [V7.0] AUTOTUNE XONG -> TAT DE BAY THUC TE
#define AUTOTUNE_MODE false

// ═══════════════════════════════════════════════════
// 1. STRUCT — khớp 100% với taycamxin.ino / drone_rc.ino
// ═══════════════════════════════════════════════════
typedef struct __attribute__((packed)) {
  int lx, ly, rx, ry;
  bool lock;
  int throttle;
} struct_command;

typedef struct __attribute__((packed)) {
  bool mpu;
  bool baro;
  bool gps;
  bool mag;
  int16_t pitch_val;
  int16_t roll_val;
  float altitude;
  int sats;
  bool isReady;
  int m1, m2, m3, m4;
  int heading;
} struct_status;

// ═══════════════════════════════════════════════════
// [V5.0 FIX #1] Double buffer cho cmd struct
// OnDataRecv (Core0) → cmd_buf → spinlock → cmd (Core1)
// ═══════════════════════════════════════════════════
static portMUX_TYPE   cmdMux    = portMUX_INITIALIZER_UNLOCKED;
static struct_command cmd_buf;           // Buffer tạm — chỉ ghi trong callback
static volatile bool  newCmdAvail = false;

struct_command cmd;   // cmd chính — CHỈ đọc/ghi dưới critical section
struct_status  fb;

MPU6050          mpu(Wire);
Adafruit_BMP280  bmp;

// ═══════════════════════════════════════════════════
// 2. PHẦN CỨNG
// ═══════════════════════════════════════════════════
const int motor_FR = 27;  // TRƯỚC PHẢI  (M1) CCW
const int motor_FL = 26;  // TRƯỚC TRÁI  (M2) CW
const int motor_RR = 14;  // SAU PHẢI    (M3) CW
const int motor_RL = 13;  // SAU TRÁI    (M4) CCW
const int motorPins[] = {motor_FR, motor_FL, motor_RR, motor_RL};
const unsigned long LOOP_TIME_US = 4000;   // 250Hz

// MAC của Remote (taycamxin / drone_rc)
uint8_t remoteMAC[] = {0xB0, 0xA6, 0x04, 0x58, 0x94, 0x48};

// ═══════════════════════════════════════════════════
// 3. PID & FILTER — V6.6 (FINAL STABLE)
// ═══════════════════════════════════════════════════
//
// [CHẨN ĐOÁN DÙA TRÊN LỊCH SỪ THUC TE]
// Từ auto-tune: Roll Ku=0.40, Pitch Ku=0.75
// Drone nhỏ nhớ -> giảm P xuống 60-70% so với Ku
// I nhỏ thôi để tránh tich luy qua nhanh
// Tuyệt đối KHÔNG dùng I-term Roll khi mới bay để dễ debug
// ═══════════════════════════════════════════════════
//                    Roll   Pitch  Yaw
float kp_inner[3] = { 0.32f, 0.45f, 1.00f };  // [V7.1] Yaw P=1.0 (Tang manh de chong xoay tron)
float ki_inner[3] = { 0.20f, 0.10f, 0.30f };  // [V7.1] Yaw I=0.30 (Giu huong khi bay)
float kd_inner[3] = { 0.025f,0.055f,0.00f };  // Yaw D=0 (khong dung D cho Yaw)
float kp_angle    = 1.50f;   // An toan: du nhanh nhung khong qua gat
float alpha_gyro  = 0.70f;   // [V7.4] Tang tu 0.50->0.70: noise thap (0.02deg), phan ung nhanh hon
float alpha_motor = 0.50f;
float alpha_d     = 0.30f;   // [V7.4] Tang tu 0.20->0.30: D-term nhanh hon, noise cho phep








// PID state
float error_inner[3]      = {0};
float integral_inner[3]   = {0};
float derivative_inner[3] = {0};
float pid_inner[3]        = {0};
float smoothGyro[3]       = {0};
// [V5.0 FIX #3] prev_gyro cho D on measurement
float prev_gyro[3]        = {0};

float angle_roll  = 0.0f;
float angle_pitch = 0.0f;

#define MAX_PID    400.0f
#define MAX_I        20.0f   // [V7.0] Giam xuong 20: I-term chi du bu lech nhe
#define MAX_THR   1000.0f
#define MIN_IDLE   120.0f    // Pulse 1120us
#define THR_I_GATE 1380      // Chi bat I-term khi ga > 1380


// Trim bù trọng tâm — dùng sau khi đã calib MPU
// [V5.3] pitch_trim_deg: reset về 0 (giá trị +5 cũ đang làm drone NGẢ RA SAU!)
// Cách dung: đặt drone nằm yên, bám "RESET MPU" trên Web UI trước khi bay
// Nếu sau calib vẫn ngả về 1 hướng: chỉnh pitch_trim_deg ±1 mỗi lần
float pitch_trim_deg =  0.0f;   // [V5.3] Reset về 0 — ngả sau là do trim +5 sai!
float roll_trim_deg  =  0.0f;   // Điều chỉnh nếu drone drift trái/phải

// Motor trim offsets (bù lệch motor/khung)
float motor_trim[4] = { 0.0f, 0.0f, 0.0f, 0.0f };  // {FR, FL, RR, RL}

// DEBUG MODE — Bật để xem chi tiết motor/gyro khi nghiêng
#define DEBUG_MOTOR 1  // 1=ON, 0=OFF

// ═══════════════════════════════════════════════════
// 4. ALTITUDE HOLD — BMP280 throttle dead zone
// ═══════════════════════════════════════════════════
#define ALT_HOLD_DZONE_LOW   1460
#define ALT_HOLD_DZONE_HIGH  1540
// [V5.1] hover_thr là biến — chỉnh được qua Web UI nút +/- không cần reflash
// Phạm vi hợp lệ: 300-900 (tương ứng 1300-1900µs pulse, hover drone 1.5kg ~62%)
#define HOVER_THR_DEFAULT    620.0f
#define HOVER_THR_MIN        300.0f   // Pulse 1300µs — thấp nhất hợp lý để giữ drone
#define HOVER_THR_MAX        900.0f   // Pulse 1900µs — cao nhất cho phép (tránh cất cánh đột ngột)
#define HOVER_THR_STEP        10.0f   // Mỗi lần nhấn +/- thay đổi 10 đơn vị
#define KP_ALT               0.60f
#define KD_ALT               0.80f
#define MAX_ALT_CORR         200.0f

float hover_thr     = HOVER_THR_DEFAULT;  // Base throttle alt-hold — lưu NVS
float alt_target    = 0.0f;
float alt_vz        = 0.0f;
float alt_thr       = HOVER_THR_DEFAULT;
bool  alt_hold_on   = false;

float seaLevelPressure = 1013.25f;

// ═══════════════════════════════════════════════════
// MOTOR TEST — An toàn khi DISARMED
// Chỉ cho phép: !isArmed, pulse ≤ MTEST_MAX_PULSE, tự tắt sau MTEST_TIMEOUT_MS
// ═══════════════════════════════════════════════════
bool          motorTestActive  = false;
unsigned long motorTestStart   = 0;
int           motorTestPulse   = 1100;    // Gia tri hien tai (us)
int           motorSoloIdx     = -1;      // [V7.2] Motor dang test don le (-1 = tat ca)
#define MTEST_MIN_PULSE   1070            // Pulse toi thieu cho test (motor vua quay)
#define MTEST_MAX_PULSE   1200            // Pulse toi da cho test (an toan, khong du lift)
#define MTEST_STEP           10           // Moi lan nhan +/- thay doi 10us
#define MTEST_TIMEOUT_MS  10000          // [V7.2] Tang len 10s cho kip quan sat

// ═══════════════════════════════════════════════════
// 5. STATE
// ═══════════════════════════════════════════════════
bool isArmed  = false;
bool mpuOK    = false;
bool baroOK   = false;

unsigned long lastRecvTime  = 0;
unsigned long lastLoopTime  = 0;
unsigned long disarmTime    = 0;
#define REARM_COOLDOWN_MS   2000

bool stick_calibrated = false;

// Soft ramp sau ARM
bool          postArmRamp  = false;
unsigned long armRampStart = 0;
#define ARM_RAMP_MS 600

// ARM timer gap tolerance
unsigned long lastGoodArmTime = 0;
int  center_rx = 1500, center_ry = 1500, center_lx = 1500;

// Chuyển pulse (us) → duty LEDC 12-bit ở 250Hz (chu kỳ 4000us)
auto pulse_to_duty = [](int pulse_us) -> uint32_t {
  return (uint32_t)((pulse_us * 4095UL) / 4000UL);
};

// ═══════════════════════════════════════════════════
// WEB CONFIGURATOR — WiFi AP + HTTP tuning UI
// SSID: Drone-Config | Pass: drone1234
// URL:  http://192.168.4.1
// ═══════════════════════════════════════════════════
WebServer webServer(80);
Preferences prefs;
#define AP_SSID "Drone-Config"
#define AP_PASS "drone1234"

// ═══════════════════════════════════════════════════
// 6. NVS LOAD/SAVE
// ═══════════════════════════════════════════════════
void loadPIDFromNVS() {
  prefs.begin("pid", true);
  kp_inner[0]    = prefs.getFloat("kp_r",  kp_inner[0]);
  kp_inner[1]    = prefs.getFloat("kp_p",  kp_inner[1]);
  kp_inner[2]    = prefs.getFloat("kp_y",  kp_inner[2]);
  ki_inner[0]    = prefs.getFloat("ki_r",  ki_inner[0]);
  ki_inner[1]    = prefs.getFloat("ki_p",  ki_inner[1]);
  ki_inner[2]    = prefs.getFloat("ki_y",  ki_inner[2]);
  kd_inner[0]    = prefs.getFloat("kd_r",  kd_inner[0]);
  kd_inner[1]    = prefs.getFloat("kd_p",  kd_inner[1]);
  kd_inner[2]    = prefs.getFloat("kd_y",  kd_inner[2]);
  kp_angle       = prefs.getFloat("kp_ang", kp_angle);
  alpha_gyro     = prefs.getFloat("a_gyro", alpha_gyro);
  alpha_motor    = prefs.getFloat("a_mot",  alpha_motor);
  alpha_d        = prefs.getFloat("a_d",    alpha_d);
  // [V7.0 FIX] KHONG LOAD TRIM TU NVS!
  // NVS co the con luu gia tri cu (+5 do) lam drone luon nga ra sau.
  // Luon bat dau o 0 de calib MPU moi co hieu qua.
  pitch_trim_deg = 0.0f;   // FORCE = 0, bo qua NVS
  roll_trim_deg  = 0.0f;   // FORCE = 0, bo qua NVS
  motor_trim[0]  = prefs.getFloat("mt0",    motor_trim[0]);
  motor_trim[1]  = prefs.getFloat("mt1",    motor_trim[1]);
  motor_trim[2]  = prefs.getFloat("mt2",    motor_trim[2]);
  motor_trim[3]  = prefs.getFloat("mt3",    motor_trim[3]);
  // [V5.1] Load hover_thr tu NVS
  hover_thr      = prefs.getFloat("hov_thr", HOVER_THR_DEFAULT);
  hover_thr      = constrain(hover_thr, HOVER_THR_MIN, HOVER_THR_MAX);
  alt_thr        = hover_thr;
  prefs.end();
  Serial.printf("[NVS] PID loaded | ptrim=FORCED_0 | hover_thr=%.0f\n", hover_thr);
}

void savePIDToNVS() {
  prefs.begin("pid", false);
  prefs.putFloat("kp_r",  kp_inner[0]);  prefs.putFloat("kp_p",  kp_inner[1]);  prefs.putFloat("kp_y",  kp_inner[2]);
  prefs.putFloat("ki_r",  ki_inner[0]);  prefs.putFloat("ki_p",  ki_inner[1]);  prefs.putFloat("ki_y",  ki_inner[2]);
  prefs.putFloat("kd_r",  kd_inner[0]);  prefs.putFloat("kd_p",  kd_inner[1]);  prefs.putFloat("kd_y",  kd_inner[2]);
  prefs.putFloat("kp_ang", kp_angle);    prefs.putFloat("a_gyro", alpha_gyro);
  prefs.putFloat("a_mot",  alpha_motor); prefs.putFloat("a_d",    alpha_d);
  prefs.putFloat("ptrim",  pitch_trim_deg); prefs.putFloat("rtrim", roll_trim_deg);
  prefs.putFloat("mt0", motor_trim[0]); prefs.putFloat("mt1", motor_trim[1]);
  prefs.putFloat("mt2", motor_trim[2]); prefs.putFloat("mt3", motor_trim[3]);
  // [V5.1] Lưu hover_thr
  prefs.putFloat("hov_thr", hover_thr);
  prefs.end();
  Serial.printf("[NVS] PID saved to flash | hover_thr=%.0f\n", hover_thr);
}

// ═══════════════════════════════════════════════════
// 7. WEB HANDLERS
// ═══════════════════════════════════════════════════
void handleSave() {
  if (isArmed) { webServer.send(403, "text/plain", "DISARM first!"); return; }
  auto gf = [&](const char* k, float def){ return webServer.hasArg(k) ? webServer.arg(k).toFloat() : def; };
  kp_inner[0] = gf("kp_r", kp_inner[0]);  kp_inner[1] = gf("kp_p", kp_inner[1]);  kp_inner[2] = gf("kp_y", kp_inner[2]);
  ki_inner[0] = gf("ki_r", ki_inner[0]);  ki_inner[1] = gf("ki_p", ki_inner[1]);  ki_inner[2] = gf("ki_y", ki_inner[2]);
  kd_inner[0] = gf("kd_r", kd_inner[0]);  kd_inner[1] = gf("kd_p", kd_inner[1]);  kd_inner[2] = gf("kd_y", kd_inner[2]);
  kp_angle       = gf("kp_ang", kp_angle);
  alpha_gyro     = gf("a_gyro", alpha_gyro);
  alpha_motor    = gf("a_mot",  alpha_motor);
  alpha_d        = gf("a_d",    alpha_d);
  pitch_trim_deg = gf("ptrim",  pitch_trim_deg);
  roll_trim_deg  = gf("rtrim",  roll_trim_deg);
  motor_trim[0]  = gf("mt0", motor_trim[0]); motor_trim[1] = gf("mt1", motor_trim[1]);
  motor_trim[2]  = gf("mt2", motor_trim[2]); motor_trim[3] = gf("mt3", motor_trim[3]);
  savePIDToNVS();
  Serial.printf("[WEB] Saved: kp_r=%.3f kp_p=%.3f kd_r=%.3f kd_p=%.3f ptrim=%.1f\n",
    kp_inner[0], kp_inner[1], kd_inner[0], kd_inner[1], pitch_trim_deg);
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleCalib() {
  if (isArmed) { webServer.send(403, "text/plain", "DISARM first!"); return; }
  Serial.println("[WEB] Bat dau Calib MPU (giu yen drone)...");
  mpu.calcOffsets(true, true);
  Serial.println("[WEB] Calib MPU hoan tat.");
  webServer.send(200, "text/plain", "OK");
}

void handleStatus() {
  // [V5.1] Trả về đầy đủ trạng thái bao gồm hover_thr và motor test
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"armed\":%s,\"roll\":%.1f,\"pitch\":%.1f,"
    "\"hov_thr\":%.0f,\"mtest\":%s,\"mpulse\":%d,"
    "\"m1\":%d,\"m2\":%d,\"m3\":%d,\"m4\":%d,\"alt\":%.2f}",
    isArmed ? "true" : "false",
    angle_roll, angle_pitch,
    hover_thr,
    motorTestActive ? "true" : "false",
    motorTestPulse,
    fb.m1, fb.m2, fb.m3, fb.m4, fb.altitude);
  webServer.send(200, "application/json", buf);
}

// ── [V5.1] HOVER THR CONTROL ─────────────────────────────────
// Tăng/giảm hover_thr theo HOVER_THR_STEP, lưu NVS ngay
// Safety: chỉ cho phép khi DISARMED, clamp trong [HOVER_THR_MIN, HOVER_THR_MAX]
void handleHoverUp() {
  if (isArmed) { webServer.send(403, "application/json", "{\"ok\":false,\"msg\":\"DISARM first\"}"); return; }
  hover_thr = constrain(hover_thr + HOVER_THR_STEP, HOVER_THR_MIN, HOVER_THR_MAX);
  alt_thr   = hover_thr;  // Sync
  prefs.begin("pid", false); prefs.putFloat("hov_thr", hover_thr); prefs.end();
  Serial.printf("[WEB] hover_thr UP -> %.0f\n", hover_thr);
  char buf[64]; snprintf(buf, sizeof(buf), "{\"ok\":true,\"hov_thr\":%.0f}", hover_thr);
  webServer.send(200, "application/json", buf);
}

void handleHoverDn() {
  if (isArmed) { webServer.send(403, "application/json", "{\"ok\":false,\"msg\":\"DISARM first\"}"); return; }
  hover_thr = constrain(hover_thr - HOVER_THR_STEP, HOVER_THR_MIN, HOVER_THR_MAX);
  alt_thr   = hover_thr;
  prefs.begin("pid", false); prefs.putFloat("hov_thr", hover_thr); prefs.end();
  Serial.printf("[WEB] hover_thr DN -> %.0f\n", hover_thr);
  char buf[64]; snprintf(buf, sizeof(buf), "{\"ok\":true,\"hov_thr\":%.0f}", hover_thr);
  webServer.send(200, "application/json", buf);
}

// ── [V5.1] MOTOR TEST CONTROL ────────────────────────────────
// Safety checklist:
//   [1] Bắt buộc DISARMED
//   [2] Pulse bị clamp cứng <= MTEST_MAX_PULSE (1200µs)
//   [3] Tự động dừng sau MTEST_TIMEOUT_MS (5s)
//   [4] Không thể ARM khi motorTestActive
void handleMotorTestUp() {
  if (isArmed) { webServer.send(403, "application/json", "{\"ok\":false,\"msg\":\"DISARM first\"}"); return; }
  motorTestPulse = constrain(motorTestPulse + MTEST_STEP, MTEST_MIN_PULSE, MTEST_MAX_PULSE);
  if (!motorTestActive) {
    motorTestActive = true;
    motorTestStart  = millis();
    Serial.printf("[MTEST] START pulse=%d (auto-stop %ds)\n", motorTestPulse, MTEST_TIMEOUT_MS/1000);
  }
  Serial.printf("[MTEST] pulse UP -> %d\n", motorTestPulse);
  char buf[64]; snprintf(buf, sizeof(buf), "{\"ok\":true,\"mpulse\":%d}", motorTestPulse);
  webServer.send(200, "application/json", buf);
}

void handleMotorTestDn() {
  if (isArmed) { webServer.send(403, "application/json", "{\"ok\":false,\"msg\":\"DISARM first\"}"); return; }
  motorTestPulse = constrain(motorTestPulse - MTEST_STEP, MTEST_MIN_PULSE, MTEST_MAX_PULSE);
  if (!motorTestActive) {
    motorTestActive = true;
    motorTestStart  = millis();
    Serial.printf("[MTEST] START pulse=%d\n", motorTestPulse);
  }
  Serial.printf("[MTEST] pulse DN -> %d\n", motorTestPulse);
  char buf[64]; snprintf(buf, sizeof(buf), "{\"ok\":true,\"mpulse\":%d}", motorTestPulse);
  webServer.send(200, "application/json", buf);
}

void handleMotorStop() {
  motorTestActive = false;
  motorTestPulse  = MTEST_MIN_PULSE;
  motorSoloIdx    = -1;
  for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], pulse_to_duty(1000));
  Serial.println("[MTEST] STOP - all motors 1000us");
  webServer.send(200, "application/json", "{\"ok\":true}");
}

// [V7.2] Test 1 motor don le
void handleMotorSolo() {
  if (isArmed) { webServer.send(403, "application/json", "{\"ok\":false,\"msg\":\"DISARM first\"}"); return; }
  if (!webServer.hasArg("m")) { webServer.send(400, "application/json", "{\"ok\":false}"); return; }
  int idx = webServer.arg("m").toInt();  // 0=M1(FR), 1=M2(FL), 2=M3(RR), 3=M4(RL)
  if (idx < 0 || idx > 3) { webServer.send(400, "application/json", "{\"ok\":false}"); return; }

  // Lay pulse tu URL neu co, neu khong dung motorTestPulse
  if (webServer.hasArg("pulse")) {
    motorTestPulse = constrain(webServer.arg("pulse").toInt(), MTEST_MIN_PULSE, MTEST_MAX_PULSE);
  }

  // Dung tat ca truoc
  for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], pulse_to_duty(1000));

  // Chi chay motor duoc chon
  motorSoloIdx    = idx;
  motorTestActive = true;
  motorTestStart  = millis();
  int pulse = constrain(motorTestPulse, MTEST_MIN_PULSE, MTEST_MAX_PULSE);
  ledcWrite(motorPins[idx], pulse_to_duty(pulse));

  const char* names[] = {"FR-Truoc Phai", "FL-Truoc Trai", "RR-Sau Phai", "RL-Sau Trai"};
  Serial.printf("[MTEST-SOLO] Motor %d (%s) pulse=%d\n", idx+1, names[idx], pulse);

  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"m\":%d,\"pulse\":%d}", idx+1, pulse);
  webServer.send(200, "application/json", buf);
}

// [V7.2] TRANG TEST MOTOR RIENG BIET
void handleMotorTestPage() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  String h = R"HTML(<!DOCTYPE html><html lang="vi"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Motor Test - Drone</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#060a10;color:#c8d8e8;font-family:'Segoe UI',sans-serif;padding:16px;max-width:420px;margin:auto;min-height:100vh}
h1{text-align:center;padding:16px 0 4px;font-size:1.4rem;background:linear-gradient(90deg,#00d2ff,#a855f7);-webkit-background-clip:text;-webkit-text-fill-color:transparent;margin-bottom:4px}
.sub{text-align:center;font-size:.72rem;color:#405060;margin-bottom:16px}
.warn{background:#2a1000;border:1px solid #ff6600;border-radius:8px;padding:10px 12px;font-size:.75rem;color:#ff9944;margin-bottom:14px;line-height:1.5}
.status{background:#0a1520;border:1px solid #1a3050;border-radius:8px;padding:10px;margin-bottom:14px;display:flex;gap:12px;align-items:center;font-size:.8rem}
.dot{width:10px;height:10px;border-radius:50%;background:#303030;flex-shrink:0;transition:background .3s}
.dot.on{background:#00ff88;box-shadow:0 0 8px #00ff88}
.dot.warn{background:#ff8800;box-shadow:0 0 8px #ff8800}
.card{background:#0d1825;border:1px solid #1a3050;border-radius:12px;padding:16px;margin-bottom:14px}
.card h2{font-size:.9rem;font-weight:700;margin-bottom:14px;color:#80b0d0}
.drone-grid{display:grid;grid-template-columns:1fr 60px 1fr;grid-template-rows:1fr 60px 1fr;gap:6px;width:280px;margin:0 auto 14px}
.mbtn{padding:14px 8px;border:2px solid #1a4060;border-radius:10px;background:#081525;color:#44aaff;font-weight:700;font-size:.85rem;cursor:pointer;transition:all .2s;line-height:1.4;text-align:center}
.mbtn:active{transform:scale(.95)}
.mbtn.active{border-color:#00d2ff;background:#001830;box-shadow:0 0 16px #00d2ff44;color:#00d2ff}
.mbtn.stopped{border-color:#1a4060;background:#081525;color:#44aaff}
.mbtn .dir{font-size:.62rem;font-weight:400;color:#608090;margin-top:3px}
.center-body{background:#0d1825;border:2px solid #1a3050;border-radius:50%;width:60px;height:60px;display:flex;align-items:center;justify-content:center;font-size:.6rem;color:#405060;text-align:center;line-height:1.3}
.arrow-tl{text-align:right;padding-right:4px;padding-bottom:4px;font-size:.65rem;color:#506070;display:flex;align-items:flex-end;justify-content:flex-end}
.arrow-tr{text-align:left;padding-left:4px;padding-bottom:4px;font-size:.65rem;color:#506070;display:flex;align-items:flex-end;justify-content:flex-start}
.arrow-bl{text-align:right;padding-right:4px;padding-top:4px;font-size:.65rem;color:#506070;display:flex;align-items:flex-start;justify-content:flex-end}
.arrow-br{text-align:left;padding-left:4px;padding-top:4px;font-size:.65rem;color:#506070;display:flex;align-items:flex-start;justify-content:flex-start}
.pulse-row{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.pulse-label{font-size:.75rem;color:#607080;min-width:60px}
.pulse-val{font-size:1.1rem;font-weight:700;color:#00d2ff;min-width:50px}
input[type=range]{flex:1;accent-color:#00d2ff}
.btn-stop{width:100%;padding:14px;background:linear-gradient(135deg,#600,#a00);border:none;border-radius:10px;color:#fff;font-size:1rem;font-weight:700;cursor:pointer;margin-top:4px;letter-spacing:.5px}
.btn-stop:active{transform:scale(.98)}
#running-info{font-size:.8rem;color:#ff8800;min-height:24px;text-align:center;margin-top:8px;padding:6px;background:#1a0f00;border-radius:6px}
.log{background:#06090f;border:1px solid #101820;border-radius:8px;padding:8px;font-size:.68rem;color:#405060;height:80px;overflow-y:auto;margin-top:10px;font-family:monospace}
a.back{display:block;text-align:center;margin-top:16px;color:#4080a0;font-size:.78rem;text-decoration:none}
a.back:hover{color:#00d2ff}
</style></head><body>
<h1>&#x1F9EA; Motor Test</h1>
<p class="sub">Nhan dien vi tri tung dong co</p>
<div class="warn">&#x26A0; THAO CANH QUAT TRUOC KHI TEST! An toan tuyet doi.</div>
<div class="status">
  <div class="dot" id="conn-dot"></div>
  <span id="conn-txt">Dang ket noi...</span>
  &nbsp;&nbsp;
  <div class="dot" id="arm-dot"></div>
  <span id="arm-txt">---</span>
</div>
<div class="card">
  <h2>&#x1F4CD; Chon Motor Can Test</h2>
  <div class="drone-grid">
    <div class="arrow-tl">TRUOC<br>TRAI &#x2196;</div>
    <div></div>
    <div class="arrow-tr">&#x2197; TRUOC<br>PHAI</div>
    <button class="mbtn" id="mb1" onclick="soloTest(1)">M2 - FL<div class="dir">CW &#x21BB;</div></button>
    <div class="center-body">DRONE<br>BODY</div>
    <button class="mbtn" id="mb0" onclick="soloTest(0)">M1 - FR<div class="dir">CCW &#x21BA;</div></button>
    <button class="mbtn" id="mb3" onclick="soloTest(3)">M4 - RL<div class="dir">CCW &#x21BA;</div></button>
    <div></div>
    <button class="mbtn" id="mb2" onclick="soloTest(2)">M3 - RR<div class="dir">CW &#x21BB;</div></button>
    <div class="arrow-bl">SAU<br>TRAI &#x2199;</div>
    <div></div>
    <div class="arrow-br">&#x2198; SAU<br>PHAI</div>
  </div>
  <div id="running-info">Chua chay motor nao</div>
</div>
<div class="card">
  <h2>&#x1F3A5; Dieu chinh Pulse</h2>
  <div class="pulse-row">
    <span class="pulse-label">Pulse:</span>
    <input type="range" id="pulse-range" min="1070" max="1200" value="1100" oninput="updatePulse(this.value)">
    <span class="pulse-val" id="pulse-disp">1100 us</span>
  </div>
  <button class="btn-stop" onclick="stopAll()">&#x23F9; DUNG TAT CA MOTOR</button>
</div>
<div class="log" id="log">Chua co log...</div>
<a class="back" href="/">&#x2190; Ve trang chinh (PID Config)</a>
<script>
const DRONE_IP = location.hostname || '192.168.4.1';
const BASE = 'http://'+DRONE_IP;
let currentMotor = -1;
let currentPulse = 1100;
const names = ['M1-FR (Truoc Phai)','M2-FL (Truoc Trai)','M3-RR (Sau Phai)','M4-RL (Sau Trai)'];
const btns = [0,1,2,3].map(i=>document.getElementById('mb'+i));

function log(msg){
  const el=document.getElementById('log');
  el.textContent=new Date().toLocaleTimeString()+' '+msg+'\n'+el.textContent;
}

function updatePulse(v){
  currentPulse=parseInt(v);
  document.getElementById('pulse-disp').textContent=v+' us';
  if(currentMotor>=0) soloTest(currentMotor);
}

function soloTest(idx){
  btns.forEach((b,i)=>{ b.className=i===idx?'mbtn active':'mbtn stopped'; });
  currentMotor=idx;
  const url=BASE+'/mtest_solo?m='+idx+'&pulse='+currentPulse;
  fetch(url)
    .then(r=>r.json())
    .then(d=>{
      if(d.ok){
        document.getElementById('running-info').textContent='&#x25B6; Dang chay: '+names[idx]+' | '+currentPulse+'us (10s)';
        log('BAT: '+names[idx]+' pulse='+currentPulse+'us');
      } else {
        document.getElementById('running-info').textContent='LOI: '+(d.msg||'unknown');
        log('LOI: '+d.msg);
        btns.forEach(b=>b.className='mbtn');
        currentMotor=-1;
      }
    })
    .catch(e=>{
      document.getElementById('running-info').textContent='Loi ket noi - Kiem tra WiFi';
      log('Loi: '+e);
    });
}

function stopAll(){
  fetch(BASE+'/mtest_stop').then(r=>r.json()).then(()=>{
    btns.forEach(b=>b.className='mbtn');
    currentMotor=-1;
    document.getElementById('running-info').textContent='Da dung tat ca motor';
    log('DUNG tat ca motor');
  });
}

// Poll status 2s
setInterval(()=>{
  fetch(BASE+'/status')
    .then(r=>r.json())
    .then(d=>{
      document.getElementById('conn-dot').className='dot on';
      document.getElementById('conn-txt').textContent='Da ket noi';
      const armed=d.armed;
      document.getElementById('arm-dot').className=armed?'dot warn':'dot';
      document.getElementById('arm-txt').textContent=armed?'ARMED - NGUY HIEM!':'DISARMED - OK';
    }).catch(()=>{
      document.getElementById('conn-dot').className='dot warn';
      document.getElementById('conn-txt').textContent='Mat ket noi';
    });
},2000);
</script></body></html>)HTML";
  webServer.send(200, "text/html", h);
}

void handleRoot() {
  String h = R"HTML(
<!DOCTYPE html><html lang="vi"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone Config V5.1</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#080c14;color:#c8d8e8;font-family:'Segoe UI',sans-serif;padding:10px;max-width:540px;margin:auto}
h1{text-align:center;padding:14px 0 2px;font-size:1.3rem;background:linear-gradient(90deg,#00d2ff,#a855f7);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;font-size:.72rem;color:#405060;margin-bottom:12px}
.sbar{display:flex;align-items:center;justify-content:center;gap:8px;background:#0d1622;border:1px solid #1e3050;border-radius:8px;padding:8px;margin-bottom:12px;font-size:.82rem;flex-wrap:wrap}
.dot{width:9px;height:9px;border-radius:50%;flex-shrink:0}
.card{background:#0d1825;border:1px solid #1a3050;border-radius:10px;padding:12px;margin-bottom:12px}
.card h2{font-size:.85rem;font-weight:700;margin-bottom:10px;color:#80b0d0;display:flex;align-items:center;gap:6px}
table{width:100%;border-collapse:collapse}
th{font-size:.7rem;color:#405060;padding:3px;text-align:center}
td{padding:3px 2px;text-align:center}
.lbl{display:inline-block;font-size:.7rem;font-weight:700;padding:1px 5px;border-radius:3px}
.r{background:#ff223322;color:#ff6677;border:1px solid #ff3344}
.p{background:#0099ee22;color:#44bbff;border:1px solid #0099ee}
.y{background:#ffaa0022;color:#ffcc44;border:1px solid #ffaa00}
input[type=number]{width:68px;background:#0a1520;border:1px solid #1e3a5f;border-radius:4px;color:#c0d8f0;font-size:.8rem;text-align:center;padding:4px}
.fl{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:6px}
.fl3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:6px}
.fld label{font-size:.7rem;color:#506070;display:block;margin-bottom:2px}
.fld input{width:100%}
.btn-save{display:block;width:100%;padding:13px;background:linear-gradient(90deg,#a855f7,#00d2ff);border:none;border-radius:8px;color:#fff;font-size:1rem;font-weight:700;cursor:pointer;margin:8px 0;transition:opacity .15s}
.btn-save:hover{opacity:.85}
.btn-save:active{transform:scale(.98)}
/* Throttle control */
.thr-panel{display:flex;align-items:center;gap:10px;background:#091520;border:1px solid #1a4060;border-radius:10px;padding:14px 12px;margin-bottom:10px}
.thr-val{flex:1;text-align:center}
.thr-num{font-size:2rem;font-weight:800;color:#00d2ff;line-height:1;font-variant-numeric:tabular-nums}
.thr-lbl{font-size:.65rem;color:#4080a0;margin-top:2px}
.thr-sub{font-size:.72rem;color:#305060;margin-top:1px}
.btn-thr{width:56px;height:56px;border-radius:10px;border:none;font-size:1.6rem;font-weight:900;cursor:pointer;transition:all .12s;display:flex;align-items:center;justify-content:center;flex-shrink:0}
.btn-up{background:linear-gradient(135deg,#1a5030,#0d8040);color:#00ff88;box-shadow:0 2px 8px #00ff8844}
.btn-up:hover{background:linear-gradient(135deg,#1f6038,#12a050);box-shadow:0 4px 12px #00ff8866}
.btn-up:active{transform:scale(.93)}
.btn-dn{background:linear-gradient(135deg,#501020,#a02030);color:#ff6080;box-shadow:0 2px 8px #ff406044}
.btn-dn:hover{background:linear-gradient(135deg,#601828,#c02840);box-shadow:0 4px 12px #ff406066}
.btn-dn:active{transform:scale(.93)}
.thr-bar-wrap{height:6px;background:#0d2030;border-radius:3px;margin-top:8px;overflow:hidden}
.thr-bar{height:100%;border-radius:3px;background:linear-gradient(90deg,#00aa44,#00d2ff);transition:width .3s}
/* Motor test */
.mtest-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.mtest-panel{background:#0a1c2a;border:1px solid #1a3a50;border-radius:8px;padding:10px;text-align:center}
.mtest-panel .mlbl{font-size:.65rem;color:#4080a0;margin-bottom:4px}
.mtest-panel .mval{font-size:1.1rem;font-weight:700;color:#c0d8f0;font-variant-numeric:tabular-nums}
.pulse-panel{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.pulse-val{flex:1;text-align:center}
.pulse-num{font-size:1.6rem;font-weight:800;color:#ffcc44;font-variant-numeric:tabular-nums}
.pulse-lbl{font-size:.65rem;color:#806020}
.btn-pulse{width:46px;height:46px;border-radius:8px;border:none;font-size:1.3rem;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:all .12s;flex-shrink:0}
.btn-pu{background:#1a3a10;color:#88ee44}
.btn-pu:hover{background:#22501a}
.btn-pd{background:#3a1a10;color:#ee8844}
.btn-pd:hover{background:#501a10}
.btn-stop{display:block;width:100%;padding:11px;background:linear-gradient(90deg,#cc2200,#ff4422);border:none;border-radius:8px;color:#fff;font-size:.95rem;font-weight:800;cursor:pointer;margin-top:8px}
.btn-stop:hover{opacity:.85}
.mtest-warn{font-size:.68rem;color:#805020;border:1px solid #402010;border-radius:6px;padding:6px 8px;margin-bottom:8px;background:#1a0c04}
.mtest-active{border-color:#ff8800!important;animation:pulse-border 1s infinite}
@keyframes pulse-border{0%,100%{box-shadow:0 0 0 0 #ff880044}50%{box-shadow:0 0 0 4px #ff880022}}
.timer-bar-wrap{height:4px;background:#1a1a0a;border-radius:2px;margin-top:6px}
.timer-bar{height:100%;border-radius:2px;background:linear-gradient(90deg,#ff4400,#ffcc00);transition:width .5s linear}
/* Guides */
.guide{font-size:.75rem}
.gi{padding:7px 0;border-bottom:1px solid #122030;display:flex;gap:8px}
.gi:last-child{border:none}
.em{font-size:1rem}
.gs{font-weight:600;color:#c0d0e0;margin-bottom:2px}
.gd{color:#607080;line-height:1.4}
.tag{font-size:.65rem;padding:1px 5px;border-radius:3px;margin-right:2px;font-weight:700}
.tg{background:#00cc6622;color:#00cc66;border:1px solid #009944}
.td{background:#ff443322;color:#ff6644;border:1px solid #cc2200}
.badge{font-size:.6rem;padding:1px 4px;border-radius:2px;background:#00cc6633;color:#00cc66;border:1px solid #00cc6666;margin-left:4px}
.sbadge{font-size:.6rem;padding:1px 5px;border-radius:3px;font-weight:700}
.sbadge-ok{background:#00cc6622;color:#00cc66;border:1px solid #009944}
.sbadge-warn{background:#ffaa0022;color:#ffaa00;border:1px solid #ffaa00}
</style></head><body>
<h1>&#x1F681; Drone Config V5.1</h1>
<p class="sub">Hover THR | Motor Test | D on Measurement | I2C 400kHz</p>
<div class="sbar" id="sbar">
  <div class="dot" id="dot" style="background:#00ff88;box-shadow:0 0 6px #00ff88"></div>
  <span id="st">Loading...</span>
  <span style="color:#203040">|</span>
  <span id="ang" style="color:#405060">R:-- P:--</span>
  <span style="color:#203040">|</span>
  <span id="altspan" style="color:#405060">Alt:--m</span>
</div>

<!-- ═══ HOVER THR CONTROL ═══ -->
<div class="card" id="hov-card">
  <h2>&#x1F680; Hover Throttle (Alt-Hold Base)
    <span class="sbadge sbadge-ok" id="hov-badge">DISARMED</span>
  </h2>
  <div class="thr-panel">
    <button class="btn-thr btn-dn" id="btn-hov-dn" onclick="hovAdj(-1)" title="Giam 10">&#x2212;</button>
    <div class="thr-val">
      <div class="thr-num" id="hov-num">)HTML";
  h += String((int)hover_thr);
  h += R"HTML(</div>
      <div class="thr-lbl">HOVER_THR &nbsp;|&nbsp; Pulse: <span id="hov-pulse">)HTML";
  h += String(1000 + (int)hover_thr);
  h += R"HTML(</span>&micro;s</div>
      <div class="thr-sub">Range )HTML";
  h += String((int)HOVER_THR_MIN);
  h += R"HTML( &ndash; )HTML";
  h += String((int)HOVER_THR_MAX);
  h += R"HTML( &nbsp;|&nbsp; Step +)HTML";
  h += String((int)HOVER_THR_STEP);
  h += R"HTML(</div>
      <div class="thr-bar-wrap"><div class="thr-bar" id="hov-bar" style="width:)HTML";
  h += String((int)((hover_thr - HOVER_THR_MIN) * 100.0f / (HOVER_THR_MAX - HOVER_THR_MIN)));
  h += R"HTML(%"></div></div>
    </div>
    <button class="btn-thr btn-up" id="btn-hov-up" onclick="hovAdj(+1)" title="Tang 10">&#x2B;</button>
  </div>
  <div style="font-size:.68rem;color:#304050;text-align:center;padding:2px 0 4px">
    &#x26A0; Chi chinh khi DISARMED &mdash; Tu dong luu NVS sau moi thay doi
  </div>
</div>

<!-- ═══ MOTOR TEST ═══ -->
<div class="card" id="mtest-card">
  <h2>&#x1F527; Motor Test (DISARMED only)
    <span class="sbadge" id="mtest-badge" style="background:#1a1a0a;color:#606060;border-color:#303030">IDLE</span>
  </h2>
  <div class="mtest-warn">
    &#x26A0; <strong>AN TOAN:</strong> Thao propeller truoc khi test! 
    Max pulse = )HTML";
  h += String(MTEST_MAX_PULSE);
  h += R"HTML(&micro;s &mdash; Tu dong dung sau )HTML";
  h += String(MTEST_TIMEOUT_MS / 1000);
  h += R"HTML(s &mdash; Bat buoc DISARMED
  </div>
  <div class="pulse-panel">
    <button class="btn-pulse btn-pd" onclick="mtAdj(-1)" title="Giam 10us">&#x2212;</button>
    <div class="pulse-val">
      <div class="pulse-num" id="mt-pulse">)HTML";
  h += String(MTEST_MIN_PULSE);
  h += R"HTML(</div>
      <div class="pulse-lbl">Motor pulse (&micro;s) &nbsp;|&nbsp; Min )HTML";
  h += String(MTEST_MIN_PULSE);
  h += R"HTML( / Max )HTML";
  h += String(MTEST_MAX_PULSE);
  h += R"HTML(</div>
      <div class="timer-bar-wrap"><div class="timer-bar" id="timer-bar" style="width:100%"></div></div>
    </div>
    <button class="btn-pulse btn-pu" onclick="mtAdj(+1)" title="Tang 10us">&#x2B;</button>
  </div>
  <div class="mtest-grid" id="motor-grid">
    <div class="mtest-panel"><div class="mlbl">FR - M1 (CCW)</div><div class="mval" id="m1v">--</div></div>
    <div class="mtest-panel"><div class="mlbl">FL - M2 (CW)</div><div class="mval" id="m2v">--</div></div>
    <div class="mtest-panel"><div class="mlbl">RR - M3 (CW)</div><div class="mval" id="m3v">--</div></div>
    <div class="mtest-panel"><div class="mlbl">RL - M4 (CCW)</div><div class="mval" id="m4v">--</div></div>
  </div>
  <button class="btn-stop" onclick="mtStop()">&#x23F9; DUNG TAT CA MOTOR</button>
</div>

<form method="GET" action="/save">
<div class="card">
  <h2>&#x26A1; Inner Loop &mdash; Rate PID</h2>
  <table><thead><tr><th></th><th>P</th><th>I</th><th>D</th></tr></thead><tbody>
  <tr><td><span class="lbl r">ROLL</span></td>
  <td><input type="number" name="kp_r" step="0.01" min="0" max="3" value=")HTML";
  h += String(kp_inner[0],3); h += R"HTML("></td>
  <td><input type="number" name="ki_r" step="0.001" min="0" max="0.5" value=")HTML";
  h += String(ki_inner[0],3); h += R"HTML("></td>
  <td><input type="number" name="kd_r" step="0.01" min="0" max="1" value=")HTML";
  h += String(kd_inner[0],3); h += R"HTML("></td></tr>
  <tr><td><span class="lbl p">PITCH</span></td>
  <td><input type="number" name="kp_p" step="0.01" min="0" max="3" value=")HTML";
  h += String(kp_inner[1],3); h += R"HTML("></td>
  <td><input type="number" name="ki_p" step="0.001" min="0" max="0.5" value=")HTML";
  h += String(ki_inner[1],3); h += R"HTML("></td>
  <td><input type="number" name="kd_p" step="0.01" min="0" max="1" value=")HTML";
  h += String(kd_inner[1],3); h += R"HTML("></td></tr>
  <tr><td><span class="lbl y">YAW</span></td>
  <td><input type="number" name="kp_y" step="0.01" min="0" max="3" value=")HTML";
  h += String(kp_inner[2],3); h += R"HTML("></td>
  <td><input type="number" name="ki_y" step="0.001" min="0" max="0.5" value=")HTML";
  h += String(ki_inner[2],3); h += R"HTML("></td>
  <td><input type="number" name="kd_y" step="0.01" min="0" max="1" value=")HTML";
  h += String(kd_inner[2],3); h += R"HTML("></td></tr>
  </tbody></table>
</div>
<div class="card">
  <h2>&#x1F3AF; Outer Loop &amp; Filters</h2>
  <div class="fl">
    <div class="fld"><label>kp_angle (Outer P)</label><input type="number" name="kp_ang" step="0.1" min="0" max="8" value=")HTML";
  h += String(kp_angle,2); h += R"HTML("></div>
    <div class="fld"><label>alpha_d (D filter) <span class="badge">D-meas</span></label><input type="number" name="a_d" step="0.001" min="0.001" max="0.3" value=")HTML";
  h += String(alpha_d,3); h += R"HTML("></div>
    <div class="fld"><label>alpha_gyro</label><input type="number" name="a_gyro" step="0.01" min="0.05" max="1" value=")HTML";
  h += String(alpha_gyro,2); h += R"HTML("></div>
    <div class="fld"><label>alpha_motor</label><input type="number" name="a_mot" step="0.01" min="0.05" max="1" value=")HTML";
  h += String(alpha_motor,2); h += R"HTML("></div>
  </div>
</div>
<div class="card">
  <h2>&#x2696; Trim &amp; Motor Offset</h2>
  <div class="fl">
    <div class="fld"><label>pitch_trim (deg)</label><input type="number" name="ptrim" step="0.5" min="-15" max="15" value=")HTML";
  h += String(pitch_trim_deg,1); h += R"HTML("></div>
    <div class="fld"><label>roll_trim (deg)</label><input type="number" name="rtrim" step="0.5" min="-15" max="15" value=")HTML";
  h += String(roll_trim_deg,1); h += R"HTML("></div>
  </div>
  <div class="fl3">
    <div class="fld"><label>Motor FR(M1)</label><input type="number" name="mt0" step="1" min="-50" max="50" value=")HTML";
  h += String(motor_trim[0],0); h += R"HTML("></div>
    <div class="fld"><label>Motor FL(M2)</label><input type="number" name="mt1" step="1" min="-50" max="50" value=")HTML";
  h += String(motor_trim[1],0); h += R"HTML("></div>
    <div class="fld"><label>Motor RR(M3)</label><input type="number" name="mt2" step="1" min="-50" max="50" value=")HTML";
  h += String(motor_trim[2],0); h += R"HTML("></div>
    <div class="fld"><label>Motor RL(M4)</label><input type="number" name="mt3" step="1" min="-50" max="50" value=")HTML";
  h += String(motor_trim[3],0); h += R"HTML("></div>
  </div>
</div>
<button type="submit" class="btn-save">&#x1F4BE; LUU PID VAO DRONE</button>
<button type="button" class="btn-save" style="background:linear-gradient(90deg,#f59e0b,#ef4444);margin-top:0;" onclick="if(confirm('Dat drone NAM YEN TREN MAT PHANG roi bam OK.')){fetch('/calib').then(r=>alert('Reset MPU ve 0 thanh cong!'));}">&#x1F3AF; RESET MPU VE 0</button>
</form>
<div class="card" id="mident-card">
<h2>&#x1F50D; NHAN DIEN TUNG MOTOR</h2>
<p style="font-size:.72rem;color:#607080;margin-bottom:10px">Bam nut de chay 1 motor (10s). Xem motor nao quay de doi chieu voi vi tri thuc te.</p>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px">
  <div style="text-align:center">
    <div style="font-size:.65rem;color:#60a0c0;margin-bottom:4px">&#x2196; TRUOC TRAI</div>
    <button onclick="soloTest(1)" id="sb1" style="width:100%;padding:12px 4px;background:#0a2030;border:2px solid #1a4060;border-radius:8px;color:#44aaff;font-weight:700;font-size:.9rem;cursor:pointer" onmousedown="this.style.opacity=.7" onmouseup="this.style.opacity=1">M2 - FL<br><span style="font-size:.65rem;font-weight:400">CW &#x21BB;</span></button>
  </div>
  <div style="text-align:center">
    <div style="font-size:.65rem;color:#60a0c0;margin-bottom:4px">TRUOC PHAI &#x2197;</div>
    <button onclick="soloTest(0)" id="sb0" style="width:100%;padding:12px 4px;background:#0a2030;border:2px solid #1a4060;border-radius:8px;color:#44aaff;font-weight:700;font-size:.9rem;cursor:pointer" onmousedown="this.style.opacity=.7" onmouseup="this.style.opacity=1">M1 - FR<br><span style="font-size:.65rem;font-weight:400">CCW &#x21BA;</span></button>
  </div>
  <div style="text-align:center">
    <button onclick="soloTest(3)" id="sb3" style="width:100%;padding:12px 4px;background:#0a2030;border:2px solid #1a4060;border-radius:8px;color:#44aaff;font-weight:700;font-size:.9rem;cursor:pointer" onmousedown="this.style.opacity=.7" onmouseup="this.style.opacity=1">M4 - RL<br><span style="font-size:.65rem;font-weight:400">CCW &#x21BA;</span></button>
    <div style="font-size:.65rem;color:#60a0c0;margin-top:4px">&#x2199; SAU TRAI</div>
  </div>
  <div style="text-align:center">
    <button onclick="soloTest(2)" id="sb2" style="width:100%;padding:12px 4px;background:#0a2030;border:2px solid #1a4060;border-radius:8px;color:#44aaff;font-weight:700;font-size:.9rem;cursor:pointer" onmousedown="this.style.opacity=.7" onmouseup="this.style.opacity=1">M3 - RR<br><span style="font-size:.65rem;font-weight:400">CW &#x21BB;</span></button>
    <div style="font-size:.65rem;color:#60a0c0;margin-top:4px">SAU PHAI &#x2198;</div>
  </div>
</div>
<div id="mident-status" style="text-align:center;font-size:.78rem;color:#ff8800;padding:6px;background:#1a1000;border-radius:6px;min-height:24px">Chua test</div>
<button onclick="mtStop()" style="width:100%;padding:8px;margin-top:8px;background:#300808;border:1px solid #601010;border-radius:6px;color:#ff6060;font-weight:700;cursor:pointer">&#x23F9; DUNG TAT CA MOTOR</button>
</div>
<script>
function soloTest(idx){
  const names=['M1-FR(Truoc Phai)','M2-FL(Truoc Trai)','M3-RR(Sau Phai)','M4-RL(Sau Trai)'];
  const btns=[document.getElementById('sb0'),document.getElementById('sb1'),document.getElementById('sb2'),document.getElementById('sb3')];
  btns.forEach((b,i)=>{ b.style.border=i===idx?'2px solid #00d2ff':'2px solid #1a4060'; b.style.background=i===idx?'#001830':'#0a2030'; });
  document.getElementById('mident-status').textContent='Dang chay: '+names[idx]+' (10 giay)';
  fetch('/mtest_solo?m='+idx).then(r=>r.json()).then(d=>{
    if(!d.ok){document.getElementById('mident-status').textContent='LOI: '+d.msg; return;}
    setTimeout(()=>{ btns.forEach(b=>{b.style.border='2px solid #1a4060';b.style.background='#0a2030';}); document.getElementById('mident-status').textContent='Da dung - bam motor khac hoac DUNG'; },10000);
  }).catch(e=>document.getElementById('mident-status').textContent='Loi ket noi');
}
</script>
<div class="card guide">
  <h2 style="margin-bottom:8px">&#x1F4CB; Huong dan Tune</h2>
  <div class="gi"><span class="em">&#x1F680;</span><div>
    <div class="gs">Hover THR la gi?</div>
    <div class="gd">Base throttle cho Alt-Hold. Bay len roi drone chim => tang. Drone cu tang len => giam.</div></div></div>
  <div class="gi"><span class="em">&#x1F525;</span><div>
    <div class="gs">Rung nhanh nho khi hover</div>
    <div class="gd"><span class="tag td">giam</span>kp_inner &nbsp; hoac &nbsp; <span class="tag tg">tang</span>kd_inner</div></div></div>
  <div class="gi"><span class="em">&#x1F30A;</span><div>
    <div class="gs">Lac cham, khong dam tat</div>
    <div class="gd"><span class="tag td">giam</span>ki_inner &nbsp; hoac &nbsp; <span class="tag tg">tang</span>kd_inner</div></div></div>
  <div class="gi"><span class="em">&#x1F422;</span><div>
    <div class="gs">Phan ung cham, lu du</div>
    <div class="gd"><span class="tag tg">tang</span>kp_inner &nbsp; hoac &nbsp; <span class="tag tg">tang</span>kp_angle</div></div></div>
  <div class="gi"><span class="em">&#x27A1;</span><div>
    <div class="gs">Drift ve mot huong lien tuc</div>
    <div class="gd">Chinh <span class="tag tg">pitch_trim</span> hoac <span class="tag tg">roll_trim</span> (+-0.5 moi lan)</div></div></div>
  <div class="gi"><span class="em">&#x1F527;</span><div>
    <div class="gs">Test motor direction</div>
    <div class="gd">Dung Motor Test panel (thao prop truoc!). Tang pulse tu 1050 len 1100 de xem huong quay.</div></div></div>
</div>
<script>
const HOV_MIN=)HTML";
  h += String((int)HOVER_THR_MIN);
  h += R"HTML(,HOV_MAX=)HTML";
  h += String((int)HOVER_THR_MAX);
  h += R"HTML(,HOV_STEP=)HTML";
  h += String((int)HOVER_THR_STEP);
  h += R"HTML(;
const MT_MIN=)HTML";
  h += String(MTEST_MIN_PULSE);
  h += R"HTML(,MT_MAX=)HTML";
  h += String(MTEST_MAX_PULSE);
  h += R"HTML(,MT_STEP=)HTML";
  h += String(MTEST_STEP);
  h += R"HTML(,MT_TIMEOUT=)HTML";
  h += String(MTEST_TIMEOUT_MS);
  h += R"HTML(;
let mtStartTime=0, mtTimerInterval=null, pollingId=null;

// ─── Hover THR buttons ───────────────────────────────────────
function hovAdj(dir){
  fetch(dir>0?'/hover_up':'/hover_dn')
    .then(r=>r.json())
    .then(d=>{
      if(!d.ok){alert('Loi: '+d.msg); return;}
      const v=d.hov_thr;
      document.getElementById('hov-num').textContent=Math.round(v);
      document.getElementById('hov-pulse').textContent=Math.round(1000+v);
      const pct=Math.round((v-HOV_MIN)*100/(HOV_MAX-HOV_MIN));
      document.getElementById('hov-bar').style.width=pct+'%';
    }).catch(e=>alert('Loi ket noi: '+e));
}

// ─── Motor Test ──────────────────────────────────────────────
function mtAdj(dir){
  fetch(dir>0?'/mtest_up':'/mtest_dn')
    .then(r=>r.json())
    .then(d=>{
      if(!d.ok){alert('Loi: '+d.msg); return;}
      document.getElementById('mt-pulse').textContent=d.mpulse;
      if(!mtStartTime){
        mtStartTime=Date.now();
        document.getElementById('mtest-card').classList.add('mtest-active');
        document.getElementById('mtest-badge').textContent='RUNNING';
        document.getElementById('mtest-badge').style.cssText='background:#ff880033;color:#ff8800;border-color:#ff8800';
        if(mtTimerInterval) clearInterval(mtTimerInterval);
        mtTimerInterval=setInterval(()=>{
          const elapsed=Date.now()-mtStartTime;
          const remain=Math.max(0,MT_TIMEOUT-elapsed);
          const pct=Math.round(remain*100/MT_TIMEOUT);
          document.getElementById('timer-bar').style.width=pct+'%';
          if(remain<=0){
            clearInterval(mtTimerInterval);
            mtTimerInterval=null;
            mtStartTime=0;
            document.getElementById('mtest-card').classList.remove('mtest-active');
            document.getElementById('mtest-badge').textContent='STOPPED(AUTO)';
            document.getElementById('mtest-badge').style.cssText='background:#1a1a0a;color:#606060;border-color:#303030';
          }
        },200);
      }
    }).catch(e=>alert('Loi ket noi: '+e));
}
function mtStop(){
  fetch('/mtest_stop')
    .then(r=>r.json())
    .then(()=>{
      mtStartTime=0;
      if(mtTimerInterval){clearInterval(mtTimerInterval);mtTimerInterval=null;}
      document.getElementById('mtest-card').classList.remove('mtest-active');
      document.getElementById('mtest-badge').textContent='STOPPED';
      document.getElementById('mtest-badge').style.cssText='background:#1a1a0a;color:#606060;border-color:#303030';
      document.getElementById('timer-bar').style.width='100%';
      document.getElementById('mt-pulse').textContent=MT_MIN;
    }).catch(e=>alert('Loi: '+e));
}

// ─── Polling 1s ──────────────────────────────────────────────
setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(d=>{
    const armed=d.armed;
    document.getElementById('dot').style.background=armed?'#ff4444':'#00ff88';
    document.getElementById('dot').style.boxShadow=armed?'0 0 6px #ff4444':'0 0 6px #00ff88';
    document.getElementById('st').textContent=armed?'ARMED':'DISARMED';
    document.getElementById('ang').textContent='R:'+d.roll.toFixed(1)+' P:'+d.pitch.toFixed(1);
    document.getElementById('altspan').textContent='Alt:'+d.alt.toFixed(2)+'m';
    // Hover THR panel
    const hovCard=document.getElementById('hov-card');
    const badge=document.getElementById('hov-badge');
    const btnUp=document.getElementById('btn-hov-up');
    const btnDn=document.getElementById('btn-hov-dn');
    if(armed){
      hovCard.style.opacity='0.5';
      badge.textContent='ARMED - khoa';
      badge.className='sbadge sbadge-warn';
      btnUp.disabled=btnDn.disabled=true;
    } else {
      hovCard.style.opacity='1';
      badge.textContent='DISARMED - OK';
      badge.className='sbadge sbadge-ok';
      btnUp.disabled=btnDn.disabled=false;
    }
    // Motor values
    document.getElementById('m1v').textContent=d.m1+'us';
    document.getElementById('m2v').textContent=d.m2+'us';
    document.getElementById('m3v').textContent=d.m3+'us';
    document.getElementById('m4v').textContent=d.m4+'us';
  }).catch(()=>{});
},1000);
</script>
</body></html>
)HTML";
  webServer.send(200, "text/html", h);
}

void setupWebServer() {
  webServer.on("/",          handleRoot);
  webServer.on("/save",      handleSave);
  webServer.on("/status",    handleStatus);
  webServer.on("/calib",     handleCalib);
  // [V5.1] Hover THR endpoints
  webServer.on("/hover_up",   handleHoverUp);
  webServer.on("/hover_dn",   handleHoverDn);
  // [V5.1] Motor Test endpoints
  webServer.on("/mtest_up",   handleMotorTestUp);
  webServer.on("/mtest_dn",   handleMotorTestDn);
  webServer.on("/mtest_stop", handleMotorStop);
  // [V7.2] Test tung motor don le
  webServer.on("/mtest_solo", handleMotorSolo);
  webServer.begin();
  Serial.printf("[WEB] Config UI: http://%s  SSID=%s\n", WiFi.softAPIP().toString().c_str(), AP_SSID);
}

// ═══════════════════════════════════════════════════
// 8. ESP-NOW CALLBACK — Core 0
// [V5.0 FIX #1] Ghi vào cmd_buf dưới spinlock
//               KHÔNG trực tiếp ghi cmd
// ═══════════════════════════════════════════════════
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(struct_command)) {
    portENTER_CRITICAL(&cmdMux);
    memcpy(&cmd_buf, data, sizeof(cmd_buf));
    newCmdAvail = true;
    portEXIT_CRITICAL(&cmdMux);

    lastRecvTime = millis();

    static unsigned long lastRecvDbg = 0;
    if (millis() - lastRecvDbg > 500) {
      lastRecvDbg = millis();
      Serial.printf("[RECV] thr=%d lock=%d\n", cmd_buf.throttle, (int)cmd_buf.lock);
    }
  } else {
    Serial.printf("[RECV] SAI SIZE: got=%d expected=%d\n", len, (int)sizeof(struct_command));
  }
}

// ═══════════════════════════════════════════════════
// 9. SENSOR SETUP — MPU6050 hardware DLPF
// ═══════════════════════════════════════════════════
void setupMPUFilter() {
  // [V7.4] DLPF = 4 → Gyro 20Hz / Accel 21Hz cutoff
  // (Truoc la 0x03=42Hz, nay giam xuong 20Hz de chong rung dong motor tot hon)
  // DLPF table:
  //   0x00 = 256Hz (KHONG LOC — gay nhieu!)
  //   0x01 = 188Hz
  //   0x02 =  98Hz
  //   0x03 =  42Hz (cu)
  //   0x04 =  21Hz  ← DUNG CAI NAY cho QAV250 rung manh
  //   0x05 =  10Hz
  //   0x06 =   5Hz  (qua cham, lam tre PID)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);   // CONFIG register
  Wire.write(0x04);   // DLPF_CFG = 4 → 21Hz cutoff
  Wire.endTransmission();
  delay(5);

  // Gyro FS_SEL=0 (±250 deg/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(5);

  // Accel FS_SEL=0 (±2g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(5);

  // --- Verify readback de dam bao DLPF da duoc ghi ---
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1);
  byte dlpfVal = Wire.read();
  Serial.printf("[MPU] DLPF verify: reg=0x%02X (mong muon=0x04)  %s\n",
    dlpfVal, dlpfVal == 0x04 ? "OK" : "FAIL! DLPF khong duoc ghi!");
  if (dlpfVal != 0x04) {
    // Thu lai lan 2
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x04);
    Wire.endTransmission();
    delay(10);
    Serial.println("[MPU] Retry ghi DLPF lan 2...");
  }
}

// ═══════════════════════════════════════════════════
// 10. ĐỌC SENSOR — MPU6050 (250Hz)
// Convention V4.3 (verified testmotoor.ino):
//   angle_pitch = +AngleX  → dương = mũi NGÓC LÊN
//   angle_roll  = +AngleY  → dương = nghiêng TRÁI
// ═══════════════════════════════════════════════════
void readSensors() {
  if (!mpuOK) return;
  mpu.update();  // [V5.0 #7] Wire timeout 3ms bảo vệ khỏi I2C hang

  // [V7.3 CRITICAL FIX] Dao dau Pitch de chong lat nguoc!
  // Neu mui ngoc len (Nose UP), AngleX duong -> can angle_pitch AM de PID day mui xuong.
  angle_pitch = -(mpu.getAngleX() + pitch_trim_deg);
  
  // [V5.3 FIX ROLL] Dao dau angle_roll
  angle_roll  = -(mpu.getAngleY() + roll_trim_deg);

  fb.pitch_val = (int16_t)angle_pitch;
  fb.roll_val  = (int16_t)angle_roll;

  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 500) {
    lastDbg = millis();
    Serial.printf("[SENSOR] roll=%.1f pitch=%.1f | GyrX=%.1f GyrY=%.1f | vz=%.2f | alt=%.2fm | armed=%d\n",
                  angle_roll, angle_pitch,
                  mpu.getGyroX(), mpu.getGyroY(),
                  alt_vz, fb.altitude, (int)isArmed);
  }
}

// ═══════════════════════════════════════════════════
// 11. ĐỌC BMP280 — Altitude + vz estimation + Alt-Hold
// ═══════════════════════════════════════════════════
void readBaro() {
  if (!baroOK) return;

  static unsigned long lastBaro = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastBaro < 62) return;
  float dt_baro = (nowMs - lastBaro) / 1000.0f;
  lastBaro = nowMs;

  float raw_alt = bmp.readAltitude(seaLevelPressure);

  static float smooth_alt = 0.0f;
  static bool  first_read = true;
  if (first_read) {
    smooth_alt = raw_alt;
    alt_target = raw_alt;
    first_read = false;
  }
  float alpha_alt = 0.15f;
  smooth_alt = alpha_alt * raw_alt + (1.0f - alpha_alt) * smooth_alt;

  static float alt_prev = 0.0f;
  float vz_raw = (smooth_alt - alt_prev) / dt_baro;
  alt_vz   = 0.10f * vz_raw + 0.90f * alt_vz;
  alt_prev = smooth_alt;

  fb.altitude = smooth_alt;

  bool in_deadzone = (cmd.throttle >= ALT_HOLD_DZONE_LOW &&
                      cmd.throttle <= ALT_HOLD_DZONE_HIGH);

  if (isArmed && in_deadzone && smooth_alt > 0.3f) {
    if (!alt_hold_on) {
      alt_target  = smooth_alt;
      alt_thr     = hover_thr;   // [V5.1] dùng biến hover_thr
      alt_hold_on = true;
      Serial.printf("[ALT-HOLD] ON | target=%.2fm\n", alt_target);
    }
    float err_alt = alt_target - smooth_alt;
    float corr    = KP_ALT * err_alt + KD_ALT * (-alt_vz);
    alt_thr = hover_thr + constrain(corr, -MAX_ALT_CORR, MAX_ALT_CORR);  // [V5.1]

  } else {
    if (alt_hold_on) {
      alt_hold_on = false;
      Serial.printf("[ALT-HOLD] OFF | alt=%.2fm vz=%.2f\n", smooth_alt, alt_vz);
    }
    // [V5.0] Đơn giản hoá: chỉ update alt_target khi armed+ngoài deadzone
    // Tránh jump khi vào deadzone lại
    if (isArmed && !in_deadzone) {
      alt_target = 0.98f * alt_target + 0.02f * smooth_alt;
    }
  }
}

// ═══════════════════════════════════════════════════
// 12. FAILSAFE & ARMING
// ═══════════════════════════════════════════════════
void safetyCheck() {
  unsigned long nowMs = millis();

  // Timeout adaptive (4s idle → 8s full throttle)
  unsigned long adaptiveTimeout = 4000 + ((cmd.throttle - 1000) / 1000.0f) * 4000;
  if (nowMs - lastRecvTime > adaptiveTimeout) {
    if (isArmed) {
      isArmed          = false;
      alt_hold_on      = false;
      postArmRamp      = false;
      stick_calibrated = false;
      Serial.printf("[FAILSAFE] TIMEOUT %lums (adaptive)\n", nowMs - lastRecvTime);
    }
    return;
  }

  // RC LOCK → disarm (cooldown 3s sau arm để tránh nhầm)
  static unsigned long armedAt = 0;
  if (isArmed && armedAt == 0) armedAt = nowMs;
  if (!isArmed) armedAt = 0;
  bool armCooldownOK = (armedAt == 0) || (nowMs - armedAt > 3000);

  if (cmd.lock && armCooldownOK) {
    if (isArmed) {
      isArmed          = false;
      alt_hold_on      = false;
      postArmRamp      = false;
      stick_calibrated = false;
      Serial.println("[FAILSAFE] LOCKED boi tam cam!");
    }
    return;
  } else if (cmd.lock && !armCooldownOK) {
    static unsigned long lastIgnoreLog = 0;
    if (nowMs - lastIgnoreLog > 1000) {
      lastIgnoreLog = nowMs;
      Serial.printf("[ARM] LOCK ignored - cooldown %lus\n", (3000-(nowMs-armedAt))/1000+1);
    }
  }

  // Debug ARM state (500ms)
  static unsigned long lastDebug = 0;
  if (nowMs - lastDebug > 500) {
    lastDebug = nowMs;
    Serial.printf("[DBG] thr=%d armed=%d alt=%.1f vz=%.2f ahl=%d\n",
                  cmd.throttle, (int)isArmed, fb.altitude, alt_vz, (int)alt_hold_on);
  }

  // Cooldown sau emergency disarm
  if (!isArmed && disarmTime > 0 && (nowMs - disarmTime < REARM_COOLDOWN_MS)) {
    return;
  }

  // ARM: lock=false + throttle thấp + sticks centered
  // [V5.1] BLOCK ARM nếu motor test đang chạy (an toàn: không ARM trong khi test motor)
  if (motorTestActive) {
    static unsigned long lastMtLog = 0;
    if (nowMs - lastMtLog > 2000) {
      lastMtLog = nowMs;
      Serial.println("[ARM] BLOCK: motorTest dang chay - nhan DUNG MOTOR truoc");
    }
    return;
  }

  bool sticksCentered = (abs(cmd.rx - center_rx) < 80 &&
                         abs(cmd.ry - center_ry) < 80 &&
                         abs(cmd.lx - center_lx) < 80);
  bool validArmCond   = (!isArmed && lastRecvTime != 0 && !cmd.lock &&
                         cmd.throttle < 1100 && sticksCentered);

  static unsigned long armRequestTime = 0;

  if (validArmCond) {
    if (armRequestTime == 0) armRequestTime = nowMs;
    lastGoodArmTime = nowMs;
    if (nowMs - armRequestTime > 600) {
      if (!stick_calibrated) {
        center_rx = (cmd.rx < 500) ? 1500 : cmd.rx;
        center_ry = (cmd.ry < 500) ? 1500 : cmd.ry;
        center_lx = (cmd.lx < 500) ? 1500 : cmd.lx;
        stick_calibrated = true;
      }
      isArmed      = true;
      disarmTime   = 0;
      postArmRamp  = true;
      armRampStart = nowMs;
      for (int i = 0; i < 3; i++) {
        integral_inner[i]   = 0;
        error_inner[i]      = 0;  // reset error state (prev_error_inner đã bỏ trong V5.0)
        derivative_inner[i] = 0;
        smoothGyro[i]       = 0;
        prev_gyro[i]        = 0;  // [V5.0 FIX #3] reset D measurement state
      }
      alt_hold_on = false;
      Serial.printf("[ARM] *** DRONE ARMED! thr=%d | Ramp %dms ***\n", cmd.throttle, ARM_RAMP_MS);
      armRequestTime  = 0;
      lastGoodArmTime = 0;
    }
  } else {
    if (armRequestTime > 0 && (nowMs - lastGoodArmTime) > 200) {
      armRequestTime  = 0;
      lastGoodArmTime = 0;
      Serial.println("[ARM] Timer reset: mat tin hieu >200ms");
    }
  }

  // Tilt failsafe: >45° HOẶC >35° + xoay nhanh >200°/s
  static unsigned long armTime = 0;
  if (isArmed  && armTime == 0) armTime = nowMs;
  if (!isArmed)                 armTime = 0;
  bool stabilized = isArmed && (nowMs - armTime > 800);

  if (stabilized) {
    bool hardTilt = (abs(angle_roll) > 60.0f || abs(angle_pitch) > 60.0f);   // [V6.6] Tang nguong cat tu 45->60 de tranh cat sớm khi luyen tap
    bool fastTilt = (abs(angle_roll) > 45.0f || abs(angle_pitch) > 45.0f) &&
                    (abs(mpu.getGyroX()) > 300.0f || abs(mpu.getGyroY()) > 300.0f);  // [V6.6] Tang nguong toc do xoay 200->300 deg/s
    if (hardTilt || fastTilt) {
      isArmed     = false;
      alt_hold_on = false;
      postArmRamp = false;
      disarmTime  = nowMs;
      Serial.printf("[FAILSAFE] %s roll=%.1f pitch=%.1f gx=%.0f gy=%.0f\n",
                    hardTilt ? "NGHIENG QUA 60deg" : "NGHIENG NHANH >45+300dps",
                    angle_roll, angle_pitch, mpu.getGyroX(), mpu.getGyroY());
    }
  }

  fb.isReady = isArmed;
}

// ═══════════════════════════════════════════════════
// 13. PID — ANGLE MODE CASCADE
//
// [V5.0 FIX #3] D on measurement:
//   raw_d = -(smoothGyro[i] - prev_gyro[i]) / dt
//   Loại bỏ derivative kick khi stick thay đổi đột ngột
//
// [V5.0 FIX #4] Anti-windup decay 0.995f → 0.9998f:
//   0.995^250  ≈ 28% sau 1s (quá hung hãng!)
//   0.9998^250 ≈ 95% sau 1s (hợp lý, giữ steady-state bù)
//
// [V5.0 FIX #5] Bỏ throttle-dependent gain scaling:
//   Gains cố định → dễ tune, điểm làm việc ổn định
// ═══════════════════════════════════════════════════
// Biến extern được khai báo ở trên, khai báo thêm ở đây để safetyCheck() dùng
float prev_error_inner[3] = {0};

void calculatePID(float dt) {
  if (!isArmed) return;

  // Outer loop: angle → rate setpoint
  float target_roll  = (abs(cmd.rx - center_rx) > 15) ? (cmd.rx - center_rx) * 0.06f : 0.0f;
  float target_pitch = (abs(cmd.ry - center_ry) > 15) ? (cmd.ry - center_ry) * 0.06f : 0.0f;

  float target_rate[3];
  target_rate[0] = kp_angle * (target_roll  - angle_roll);
  target_rate[1] = kp_angle * (target_pitch - angle_pitch);
  // Yaw deadzone 40us để tránh pid2 max khi stick trung tâm bị lệch nhẹ
  target_rate[2] = (abs(cmd.lx - center_lx) > 40) ? (cmd.lx - center_lx) * 0.60f : 0.0f;

  // [V6.2 + V7.3 FIX] rawGyr:
  // Roll  (Y): Dao dau de khop voi angle_roll
  // Pitch (X): Dao dau de khop voi angle_pitch (chong lat nguoc)
  // Yaw   (Z): Giu nguyen, Right-hand rule (CCW = duong) la dung cho Yaw mixer hien tai
  float rawGyr[3];
  rawGyr[0] = -mpu.getGyroY();   // Roll
  rawGyr[1] = -mpu.getGyroX();   // [FIX V7.3] Pitch phai dao dau!
  rawGyr[2] = +mpu.getGyroZ();   // Yaw

  // Yaw debug (1s)
  static unsigned long lastYawDbg = 0;
  if (millis() - lastYawDbg > 1000) {
    lastYawDbg = millis();
    Serial.printf("[YAW] GyroZ_raw=%.1f tgt=%.1f err=%.1f | pid2=%.1f | M1=%d M2=%d M3=%d M4=%d\n",
      mpu.getGyroZ(), target_rate[2],
      target_rate[2] - smoothGyro[2],
      pid_inner[2],
      fb.m1, fb.m2, fb.m3, fb.m4);
  }

  for (int i = 0; i < 3; i++) {
    // EMA filter gyro
    smoothGyro[i] = alpha_gyro * rawGyr[i] + (1.0f - alpha_gyro) * smoothGyro[i];
    error_inner[i] = target_rate[i] - smoothGyro[i];

    // I-term: tích khi throttle đủ cao hoặc alt-hold
    bool ok_to_integrate = (cmd.throttle > THR_I_GATE) || alt_hold_on;
    if (ok_to_integrate) {
      integral_inner[i] = constrain(
        integral_inner[i] + error_inner[i] * dt,
        -MAX_I, MAX_I
      );
      // [V5.2 FIX] Anti-windup: decay nhanh hơn khi error nhỏ (drone gần ổn định)
      // 0.995^250Hz = 28% sau 1s → quá hung hãng
      // 0.9998^250Hz = 95% sau 1s → tích lũy gây drift kéo dài!
      // Giải pháp: decay theo biên độ error
      float decay = (abs(error_inner[i]) < 2.0f)  ? 0.990f :   // Gần center: decay nhanh
                    (abs(error_inner[i]) < 8.0f)  ? 0.9985f :  // Xa vừa: decay trung bình
                                                     0.9998f;   // Sai số lớn: giữ I
      integral_inner[i] *= decay;
    } else {
      // Dưới THR_I_GATE: xả I-term nhanh về 0 (tránh windup khi hover thấp)
      integral_inner[i] *= 0.95f;
    }

    // [V5.0 FIX #3] D on measurement thay D on error
    // → Loại bỏ derivative kick khi setpoint (target_rate) thay đổi đột ngột
    // → Dấu âm: khi gyro tăng (đang xoay mạnh hơn), D cản lại
    float raw_d = -(smoothGyro[i] - prev_gyro[i]) / dt;
    derivative_inner[i] = (1.0f - alpha_d) * derivative_inner[i] + alpha_d * raw_d;
    prev_gyro[i] = smoothGyro[i];  // Cập nhật prev_gyro (không phải prev_error)

    // [V5.0 FIX #5] Gains cố định — KHÔNG nhân thr_scale
    pid_inner[i] = constrain(
      kp_inner[i] * error_inner[i]      +
      ki_inner[i] * integral_inner[i]   +
      kd_inner[i] * derivative_inner[i],
      -MAX_PID, MAX_PID
    );

    // Giữ prev_error cho tham chiếu debug (không dùng trong PID nữa)
    prev_error_inner[i] = error_inner[i];
  }

  // Debug PID (mỗi 2s)
  static unsigned long lastPidDbg = 0;
  if (millis() - lastPidDbg > 2000) {
    lastPidDbg = millis();
    Serial.printf("[PID] thr=%d | kP: R=%.2f P=%.2f Y=%.2f | pid: R=%.1f P=%.1f Y=%.1f\n",
      cmd.throttle,
      kp_inner[0], kp_inner[1], kp_inner[2],
      pid_inner[0], pid_inner[1], pid_inner[2]);
  }
}

// ═══════════════════════════════════════════════════
// 14. MOTOR MIXER — Quad-X (Sign Paradox Fix V4.3)
//
// [SIGN PARADOX LOGIC]
//   Error = Target(0) - Current(+angle) = ÂM khi nghiêng
//   pid_inner mang giá trị ÂM → cần correction
//   "thr - pid" = tăng motor khi pid âm → đúng
//
// [Convention testmotoor.ino]:
//   AngleX>0 → nâng FR+FL | AngleY>0 → nâng FL+RL
//
//   FR-M1(27): thr - pid[0] - pid[1] - pid[2]  (CCW)
//   FL-M2(26): thr + pid[0] - pid[1] + pid[2]  (CW)
//   RR-M3(14): thr - pid[0] + pid[1] + pid[2]  (CW)
//   RL-M4(13): thr + pid[0] + pid[1] - pid[2]  (CCW)
//
// [V5.0 FIX #2] Reset smooth_m[] khi disarm
// ═══════════════════════════════════════════════════
void applyMixer() {
  // [V5.0 FIX #2] static smooth_m: khai báo 1 lần, nhưng reset khi disarm
  static float smooth_m[4] = {MIN_IDLE, MIN_IDLE, MIN_IDLE, MIN_IDLE};

  // ──────────────────────────────────────────────────────
  // [V5.1] MOTOR TEST MODE — Chế độ test an toàn (chỉ khi DISARMED)
  // Safety guarantees:
  //   • Bắt buộc !isArmed (hàm này return sớm nếu isArmed)
  //   • Pulse bị clamp cứng ≤ 1000 + MTEST_MAX_PULSE (1200µs)
  //   • Tự động dừng sau MTEST_TIMEOUT_MS (5 giây)
  // ──────────────────────────────────────────────────────
  if (motorTestActive && !isArmed) {
    // Auto-stop sau timeout
    if (millis() - motorTestStart >= MTEST_TIMEOUT_MS) {
      motorTestActive = false;
      motorTestPulse  = MTEST_MIN_PULSE;
      for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], pulse_to_duty(1000));
      fb.m1 = fb.m2 = fb.m3 = fb.m4 = 1000;
      Serial.println("[MTEST] AUTO-STOP (5s timeout)");
      return;
    }
    // Clamp cứng: dù web gửi giá trị nào, pulse không vượt MTEST_MAX_PULSE
    int safePulse = constrain(motorTestPulse, MTEST_MIN_PULSE, MTEST_MAX_PULSE);
    for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], pulse_to_duty(safePulse));
    fb.m1 = fb.m2 = fb.m3 = fb.m4 = safePulse;
    // Log mỗi 1s
    static unsigned long lastMtLog2 = 0;
    if (millis() - lastMtLog2 > 1000) {
      lastMtLog2 = millis();
      unsigned long remain = (MTEST_TIMEOUT_MS - (millis() - motorTestStart)) / 1000;
      Serial.printf("[MTEST] RUNNING pulse=%d | auto-stop in %lus\n", safePulse, remain);
    }
    return;  // Return sớm — không chạy PID mixer bên dưới
  }

  if (!isArmed) {
    // [V5.0 FIX #2] Reset về MIN_IDLE khi disarm
    for (int i = 0; i < 4; i++) smooth_m[i] = MIN_IDLE;
    for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], pulse_to_duty(1000));
    fb.m1 = fb.m2 = fb.m3 = fb.m4 = 1000;
    alt_thr     = hover_thr;  // [V5.1] dùng hover_thr biến
    postArmRamp = false;
    return;
  }

  // Throttle: ALT-HOLD hoặc MANUAL
  float thr;
  if (alt_hold_on) {
    // [V5.1] dùng hover_thr biến (không phải HOVER_THR constant nữa)
    thr = constrain(alt_thr, MIN_IDLE, MAX_THR);
  } else {
    // Linear map: 1000µs→MIN_IDLE, 2000µs→MAX_THR (không có dead zone ẩn)
    thr = MIN_IDLE + (float)(cmd.throttle - 1000) * (MAX_THR - MIN_IDLE) / 1000.0f;
    thr = constrain(thr, MIN_IDLE, MAX_THR);
  }

  // Quad-X mixer (Sign Paradox Fix V4.3)
  float m0 = thr - pid_inner[0] - pid_inner[1] - pid_inner[2]; // FR (M1) CCW
  float m1 = thr + pid_inner[0] - pid_inner[1] + pid_inner[2]; // FL (M2) CW
  float m2 = thr - pid_inner[0] + pid_inner[1] + pid_inner[2]; // RR (M3) CW
  float m3 = thr + pid_inner[0] + pid_inner[1] - pid_inner[2]; // RL (M4) CCW

  float m[4] = {m0, m1, m2, m3};

  // Motor trim offset (bù lệch cơ học)
  for (int i = 0; i < 4; i++) m[i] += motor_trim[i];

  // (Đã xóa bù +60 cơ học vì nó gây lướt sóng, để I-term tự lo)

  // Shift thay vì Scale — bảo toàn hiệu motor, tránh thrust lurch
  float m_max = max({m[0], m[1], m[2], m[3]});
  if (m_max > MAX_THR) {
    float excess = m_max - MAX_THR;
    for (int i = 0; i < 4; i++) m[i] -= excess;
  }
  for (int i = 0; i < 4; i++) m[i] = max(m[i], MIN_IDLE);

  // Soft ramp sau ARM: tăng dần MIN_IDLE → target trong ARM_RAMP_MS
  if (postArmRamp) {
    float rampPct = constrain(
      (float)(millis() - armRampStart) / (float)ARM_RAMP_MS, 0.0f, 1.0f);
    for (int i = 0; i < 4; i++) {
      m[i] = MIN_IDLE + (m[i] - MIN_IDLE) * rampPct;
    }
    if (rampPct >= 1.0f) {
      postArmRamp = false;
      Serial.println("[RAMP] ARM soft-ramp complete");
    }
  }

  // EMA output smoothing
  for (int i = 0; i < 4; i++) {
    smooth_m[i] = smooth_m[i] * (1.0f - alpha_motor) + m[i] * alpha_motor;
  }

  // Ghi PWM
  for (int i = 0; i < 4; i++) {
    int pulse = 1000 + (int)constrain(smooth_m[i], 0.0f, MAX_THR);
    ledcWrite(motorPins[i], pulse_to_duty(pulse));
    if (i == 0) fb.m1 = pulse;
    if (i == 1) fb.m2 = pulse;
    if (i == 2) fb.m3 = pulse;
    if (i == 3) fb.m4 = pulse;
  }

  // Debug motor (2s)
  static unsigned long lastMDbg = 0;
  if (millis() - lastMDbg > 2000) {
    lastMDbg = millis();
    Serial.printf("[MOTOR] thr=%.0f M1:%d M2:%d M3:%d M4:%d ahl=%d\n",
                  thr, fb.m1, fb.m2, fb.m3, fb.m4, (int)alt_hold_on);
#if DEBUG_MOTOR
    Serial.printf("[DEBUG] angle_roll=%.1f° angle_pitch=%.1f° | ",
                  angle_roll, angle_pitch);
    Serial.printf("pid[R]=%.1f pid[P]=%.1f pid[Y]=%.1f\n",
                  pid_inner[0], pid_inner[1], pid_inner[2]);
#endif
  }
}

// ═══════════════════════════════════════════════════
// 15. I2C BUS RECOVERY — giải phóng bus bị treo (SDA stuck LOW)
// ═══════════════════════════════════════════════════
void i2cBusRecovery() {
  pinMode(21, OUTPUT); // SDA
  pinMode(22, OUTPUT); // SCL
  digitalWrite(21, HIGH);
  digitalWrite(22, HIGH);
  delayMicroseconds(5);

  for (int i = 0; i < 9; i++) {
    digitalWrite(22, LOW);
    delayMicroseconds(5);
    digitalWrite(22, HIGH);
    delayMicroseconds(5);
    if (digitalRead(21) == HIGH) break;
  }
  // STOP condition
  digitalWrite(21, LOW);
  delayMicroseconds(5);
  digitalWrite(22, HIGH);
  delayMicroseconds(5);
  digitalWrite(21, HIGH);
  delayMicroseconds(5);

  // [V5.0 FIX #6] Trả lại Wire ở 400kHz
  Wire.begin(21, 22);
  Wire.setClock(400000);
  // [V5.0 FIX #7] Wire timeout
  Wire.setTimeOut(3);
  delayMicroseconds(500);
}

// ═══════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Drone Firmware V5.0");
  Serial.println("  Fixes: Race-condition cmd | smooth_m reset | D-on-measurement");
  Serial.println("         Anti-windup decay | I2C 400kHz | Wire timeout");

  // ── Khởi tạo I2C TRƯỚC khi arm ESC ──────────────────────
  Wire.begin(21, 22);
  // [V5.0 FIX #6] I2C 100kHz → 400kHz (GY-87 fast-mode support)
  // Tiết kiệm ~16% CPU budget so với 100kHz
  Wire.setClock(400000);
  // [V5.0 FIX #7] Wire timeout 3ms: tránh mpu.update() block khi I2C hang
  Wire.setTimeOut(3);
  delay(100);

  // ── ESC setup: OUTPUT + 1000µs ngay (chưa delay arm) ─────
  for (int i = 0; i < 4; i++) {
    ledcAttach(motorPins[i], 250, 12);
    ledcWrite(motorPins[i], pulse_to_duty(1000));
  }
  Serial.println("[ESC] PWM 1000us started (chua arm - doi MPU init xong)");

  // ── MPU6050 — Retry 5 lần với I2C recovery ───────────────
  int mpuRet = -1;
  for (int attempt = 1; attempt <= 5 && mpuRet != 0; attempt++) {
    mpuRet = mpu.begin();
    Serial.printf("[MPU] begin() attempt %d = %d (0=OK)\n", attempt, mpuRet);
    if (mpuRet != 0 && attempt < 5) {
      Serial.println("[MPU] Retry sau 800ms... (bus recovery)");
      i2cBusRecovery();
      delay(800);
    }
  }

  if (mpuRet == 0) {
    delay(300);

    // Kiểm tra MPU awake (PWR_MGMT_1)
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1);
    byte pwr = Wire.read();
    Serial.printf("[MPU] PWR_MGMT_1=0x%02X sleep=%d\n", pwr, (pwr >> 6) & 1);
    if ((pwr >> 6) & 1) {
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission();
      delay(100);
      Serial.println("[MPU] Force wake-up!");
    }

    // calcOffsets() TRƯỚC setupMPUFilter() (thư viện reset config bên trong)
    Serial.println("[MPU] Calibrating... DAT DRONE PHANG TREN MAT PHANG!");
    delay(1000);
    mpu.calcOffsets();

    // SAU calcOffsets(): apply DLPF filter
    setupMPUFilter();
    delay(50);

    // Verify data không bị frozen
    mpu.update();
    float testGx = mpu.getGyroX();
    float testGy = mpu.getGyroY();
    float testAx = mpu.getAccX();
    Serial.printf("[MPU] Verify: AccX=%.2f GyrX=%.2f GyrY=%.2f\n", testAx, testGx, testGy);
    if (testAx == 0.0f && testGx == 0.0f && testGy == 0.0f) {
      Serial.println("[ERR] MPU data ALL ZERO! Sensor frozen - kiem tra I2C!");
      while(1) { delay(500); }
    }

    mpuOK  = true;
    fb.mpu = true;
    Serial.printf("[OK] MPU6050 ready @ 400kHz | pitch_trim=+%.1fdeg\n", pitch_trim_deg);
  } else {
    Serial.println("[ERR] MPU6050 FAILED! Kiem tra day SDA/SCL va nguon 3.3V");
    while(1) { delay(500); }
  }

  // ── ESC arm delay THỰC SỰ — sau khi MPU đã OK ────────────
  Serial.println("[ESC] Arm sequence: giu 1000us trong 2s...");
  delay(2000);
  Serial.println("[ESC] Arm sequence done!");

  // ── BMP280 ────────────────────────────────────────────────
  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X4,
      Adafruit_BMP280::SAMPLING_X4,
      Adafruit_BMP280::FILTER_X8,
      Adafruit_BMP280::STANDBY_MS_63
    );
    baroOK      = true;
    fb.baro     = true;
    delay(200);
    float base = bmp.readAltitude(seaLevelPressure);
    fb.altitude = base;
    Serial.printf("[OK] BMP280 | Base alt=%.2fm | Alt-Hold zone %d-%d\n",
                  base, ALT_HOLD_DZONE_LOW, ALT_HOLD_DZONE_HIGH);
  } else {
    baroOK  = false;
    fb.baro = false;
    Serial.println("[WARN] BMP280 not found - no alt-hold");
  }

  fb.gps = false; fb.mag = false;
  fb.sats = 0;    fb.heading = 0;

  // ── WiFi + ESP-NOW ────────────────────────────────────────
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  delay(200);

  bool apOK = WiFi.softAP(AP_SSID, AP_PASS, 1);
  delay(500);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("[AP] %s | IP: %s | OK=%d\n", AP_SSID, apIP.toString().c_str(), (int)apOK);
  Serial.printf("[AP] >>> Ket noi WiFi: %s / Pass: %s <<<\n", AP_SSID, AP_PASS);
  Serial.printf("[AP] >>> Mo browser: http://%s <<<\n", apIP.toString().c_str());

  loadPIDFromNVS();

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, remoteMAC, 6);
    peer.channel = 1;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println("[OK] ESP-NOW CH1");
    Serial.printf("  MAC DRONE NAY : %s\n", WiFi.macAddress().c_str());
    Serial.printf("  REMOTE (RC)   : %02X:%02X:%02X:%02X:%02X:%02X\n",
      remoteMAC[0], remoteMAC[1], remoteMAC[2],
      remoteMAC[3], remoteMAC[4], remoteMAC[5]);
  } else {
    Serial.println("[ERR] ESP-NOW FAILED!");
  }

  setupWebServer();

  lastLoopTime = micros();
  Serial.println("[BOOT] Ready! ARM = giu nut 2s tren tay cam + ga thap");
  Serial.printf("  Trim: pitch=+%.1f deg | hover_thr=%.0f\n", pitch_trim_deg, hover_thr);
  Serial.println("  V5.0 Fixes active: race-cond / smooth_m / D-meas / anti-windup / I2C400k / timeout");
}

// ═══════════════════════════════════════════════════
// LOOP — 250Hz
// ═══════════════════════════════════════════════════
void loop() {
  unsigned long now = micros();
  if (now - lastLoopTime < LOOP_TIME_US) return;

  float dt = (now - lastLoopTime) / 1000000.0f;
  dt = constrain(dt, 0.001f, 0.020f);
  lastLoopTime = now;

  // ── [V5.0 FIX #1] Copy cmd_buf → cmd dưới critical section ──
  // OnDataRecv (Core0) ghi cmd_buf; loop() (Core1) copy vào cmd
  // portMUX spinlock đảm bảo atomic read/write
  portENTER_CRITICAL(&cmdMux);
  if (newCmdAvail) {
    cmd        = cmd_buf;
    newCmdAvail = false;
  }
  portEXIT_CRITICAL(&cmdMux);
  // ─────────────────────────────────────────────────────────────

  readSensors();      // MPU6050 (250Hz) + Wire timeout 3ms
  readBaro();         // BMP280 (16Hz) + alt-hold calc
  safetyCheck();      // Failsafe + ARM logic
  if (!isArmed) webServer.handleClient();  // Web config chỉ khi disarmed
  calculatePID(dt);   // Cascade PID (D on measurement, gains cố định)
  applyMixer();       // Motor output (smooth_m reset khi disarm)

  // Telemetry → RC (10Hz)
  static unsigned long lastTelem = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastTelem >= 100) {
    lastTelem  = nowMs;
    fb.isReady = isArmed;
    fb.mpu     = mpuOK;
    fb.baro    = baroOK;
    esp_now_send(remoteMAC, (uint8_t*)&fb, sizeof(fb));
  }

#if AUTOTUNE_MODE
  // Nhan lenh P tu Python qua Serial - CHI THAY DOI ROLL (index 0)
  // Pitch giu nguyen gia tri da tune o 0.45 de tranh lắc truoc sau!
  if (Serial.available()) {
    String cmdStr = Serial.readStringUntil('\n');
    if (cmdStr.startsWith("P:")) {
      float newP = cmdStr.substring(2).toFloat();
      kp_inner[0] = newP;  // <-- CHI ROLL, Pitch KHONG THAY DOI
      Serial.printf("=> ROLL P SET TO: %.2f | PITCH P FIXED=%.2f\n", kp_inner[0], kp_inner[1]);
    }
  }

  // Gui Telemetry toc do cao cho truc ROLL
  static unsigned long lastAutoTune = 0;
  if (nowMs - lastAutoTune >= 20) {
    lastAutoTune = nowMs;
    // "r" = angle_roll (goc nghieng TRAI/PHAI)
    Serial.printf("{\"t\":%lu,\"r\":%.2f,\"p\":%.2f,\"m\":%d}\n", 
      nowMs, angle_roll, kp_inner[0], fb.m1);
  }
#endif
}