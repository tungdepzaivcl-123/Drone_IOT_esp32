/*
 * ============================================================
 *  REMOTE CONTROL V4.4 — ESP-NOW + MQTT REALTIME
 *
 *  [FIX v4.4] - QUAN TRONG!
 *  - WiFi + MQTT kết nối NGAY từ boot (không chờ 2 phút)
 *  - ESP-NOW và WiFi chạy song song trên cùng kênh
 *  - MQTT gửi telemetry REALTIME khi đang bay (500ms/lần)
 *  - KHÔNG cắt WiFi khi ARM nữa
 *  - Tự động reconnect MQTT nếu mất kết nối
 *
 *  [KEEP] ESP-NOW kênh 1 | ARM/LOCK giữ nút ≥0.8s
 *  [KEEP] Chuyển trang nhấn ngắn | EMA joystick
 * ============================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_wifi.h>
#include <PubSubClient.h>

// ── WiFi & MQTT ──────────────────────────────────────────────
const char* ssid        = "KS_SINH VIEN";
const char* password    = "kssv@2025";
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic  = "hagiang/drone/telemetry";
const char* mqtt_id     = "Alpha_K25_Remote";

// ── STRUCT (khớp 100% với drone) ─────────────────────────────
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

struct_command cmd;
struct_status  fb;

// ── MAC DRONE ─────────────────────────────────────────────────
uint8_t droneMAC[] = {0x94, 0x51, 0xDC, 0x4B, 0x81, 0x38};

// ── JOYSTICK PINS ─────────────────────────────────────────────
#define PIN_JOY_THR   35
#define PIN_JOY_YAW   34
#define PIN_JOY_PIT   33
#define PIN_JOY_ROLL  32
#define PIN_BTN_LOCK  26

// ── LCD ───────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define TOTAL_PAGES 6
int  displayPage = 0;
bool needClear   = false;
unsigned long armMsgUntil = 0;

// ── STATE ─────────────────────────────────────────────────────
bool isLocked       = true;
bool droneConnected = false;
bool droneDisplay   = false;

// [FIX] pendingArm: RC da mo khoa nhung dang cho drone xac nhan ARM
// Trong thoi gian nay gui throttle=1000 de drone du dieu kien ARM
// Chi cho phep ga that khi fb.isReady = true
bool          pendingArm      = false;
unsigned long armPendingStart = 0;
#define ARM_CONFIRM_TIMEOUT_MS  4000   // 4s timeout: neu drone khong xac nhan → relock

#define DRONE_TIMEOUT_MS      8000
#define DRONE_DISPLAY_DELAY   3000

unsigned long lastDroneRecvTime  = 0;
unsigned long droneDisconnectAt  = 0;

// ── BUTTON ────────────────────────────────────────────────────
#define HOLD_MS 2000  // Giảm từ 800ms → 2000ms để dễ nhấn, tránh nhầm
static bool          lastBtn       = HIGH;
static unsigned long btnPressStart = 0;
static bool          holdFired     = false;

// ── EMA JOYSTICK ──────────────────────────────────────────────
float smooth_thr  = 1000;
float smooth_yaw  = 1500;
float smooth_pit  = 1500;
float smooth_roll = 1500;
const float ALPHA_RC = 0.20f;

// [FIX] Yaw joystick center calibration
// Joystick neutral VL khong phai ADC midpoint (2048) -> phai doc tai boot
// Neu KHÔNG doc: yaw luc nao cung lech -> pid2 max -> M1/M4 nong
int yaw_neutral_adc = 2048;  // Se doc lai trong setup()
#define YAW_DEADZONE_US 80   // Deadzone yaw tren RC (us) - lon de an toan

// ── ESP-NOW TIMING ────────────────────────────────────────────
static unsigned long lastSendTime = 0;
#define SEND_INTERVAL_MS 20   // 50Hz

// ── WIFI / MQTT STATE ─────────────────────────────────────────
WiFiClient    espClient;
PubSubClient  mqttClient(espClient);

bool          wifiOK          = false;
unsigned long lastMQTTAttempt = 0;
unsigned long lastMQTTSend    = 0;
unsigned long lastWiFiCheck   = 0;

// ── ĐỌC ADC TRUNG BÌNH ────────────────────────────────────────
// [V4.5 FIX] 20→8 mẫu: giảm blocking ~2ms→0.8ms per pin (4 pins = 3.2ms tiết kiệm/loop)
// EMA ALPHA_RC=0.20 đã đủ smooth, không cần lấy 20 mẫu
int readSmooth(int pin) {
  long s = 0;
  for (int i = 0; i < 8; i++) s += analogRead(pin);
  return (int)(s / 8);
}

// ── HELPER LCD ────────────────────────────────────────────────
void lcdPrintf(int col, int row, const char* fmt, ...) {
  char buf[17];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  lcd.setCursor(col, row);
  lcd.print(buf);
}

// ── ESP-NOW RECV ──────────────────────────────────────────────
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(fb)) {
    memcpy(&fb, data, sizeof(fb));
    lastDroneRecvTime = millis();
    if (!droneConnected) {
      droneConnected = true;
      Serial.println("[ESP-NOW] Drone found!");
    }
    if (!droneDisplay) {
      droneDisplay = true;
      needClear = true;
    }
    // Debug mỗi 1s
    static unsigned long lastFbDbg = 0;
    if (millis() - lastFbDbg > 1000) {
      lastFbDbg = millis();
      Serial.printf("[TELEM] alt=%.1f roll=%d pitch=%d m1=%d m2=%d m3=%d m4=%d\n",
                    fb.altitude, (int)fb.roll_val, (int)fb.pitch_val,
                    fb.m1, fb.m2, fb.m3, fb.m4);
    }
  } else {
    Serial.printf("[RECV] SAI SIZE: got=%d expected=%d\n", len, (int)sizeof(fb));
  }
}

// ── QUẢN LÝ WIFI ─────────────────────────────────────────────
// Gọi từ loop() — không blocking, không delay()
void handleWiFi(unsigned long now) {
  // Chỉ kiểm tra mỗi 2s để không tốn thời gian
  if (now - lastWiFiCheck < 2000) return;
  lastWiFiCheck = now;

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiOK) {
      wifiOK = false;
      Serial.println("[WiFi] Mat ket noi — dang reconnect...");
    }
    WiFi.reconnect();
  } else if (!wifiOK) {
    wifiOK = true;
    Serial.print("[WiFi] Connected: ");
    Serial.println(WiFi.localIP());
  }
}

// ── QUẢN LÝ MQTT ─────────────────────────────────────────────
void handleMQTT(unsigned long now) {
  if (!wifiOK) return;

  if (!mqttClient.connected()) {
    if (now - lastMQTTAttempt < 5000) return;  // Thử lại mỗi 5s
    lastMQTTAttempt = now;
    Serial.print("[MQTT] Connecting...");
    if (mqttClient.connect(mqtt_id)) {
      Serial.println(" OK!");
    } else {
      Serial.printf(" FAIL rc=%d\n", mqttClient.state());
      return;
    }
  }
  mqttClient.loop();
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  pinMode(PIN_BTN_LOCK, INPUT_PULLUP);

  // ─── WiFi: STA mode, kênh 1, kết nối ngay ──────────────────
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);         // Tự reconnect nếu mất mạng
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);       // Tắt power save → giảm latency ESP-NOW
  delay(100);

  // [CRITICAL] Kênh 1 — ESP-NOW và WiFi dùng chung kênh này
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Bắt đầu kết nối WiFi (non-blocking)
  WiFi.begin(ssid, password);
  Serial.println("[WiFi] Connecting...");

  // ─── ESP-NOW: khởi tạo sau WiFi.begin() ────────────────────
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, droneMAC, 6);
    peer.channel = 1;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("[OK] ESP-NOW ready (CH1)");
    Serial.printf("  Drone MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      droneMAC[0],droneMAC[1],droneMAC[2],droneMAC[3],droneMAC[4],droneMAC[5]);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("ESP-NOW FAILED!");
    while (1);
  }

  // ─── MQTT: chỉ cấu hình server ─────────────────────────────
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setKeepAlive(30);

  lcd.clear();
  lcdPrintf(0, 0, "RC V4.4 READY!");
  lcdPrintf(0, 1, "Cho drone...");

  // [FIX] Doc yaw joystick center tai boot (truoc khi user cham stick)
  // Day la fix chinh cho bug pid2=-300: joystick neutral != ADC 2048
  {
    long s = 0;
    for (int i = 0; i < 30; i++) { s += analogRead(PIN_JOY_YAW); delay(5); }
    yaw_neutral_adc = (int)(s / 30);
    Serial.printf("[CAL] yaw_neutral_adc=%d (map->%d us)\n",
      yaw_neutral_adc,
      (int)map(yaw_neutral_adc, 0, 4095, 1000, 2000));
  }

  delay(800);
  lcd.clear();
}

// ─────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── 1. ĐỌC JOYSTICK + EMA ────────────────────────────────
  int raw_thr   = constrain(map(readSmooth(PIN_JOY_THR), 150, 3950, 1000, 2000), 1000, 2000);
  // [FIX] Yaw: map relative to calibrated neutral, khong dung 0-4095 fixed
  // Truoc: map(adc, 0,4095, 1000,2000) -> neu neutral=61, yaw=1015 (lech 485us!)
  // Sau:   1500 + (adc - neutral) * 500/2048 -> neutral luon = 1500
  int raw_yaw_adc = readSmooth(PIN_JOY_YAW);
  int raw_yaw   = constrain(1500 + (raw_yaw_adc - yaw_neutral_adc) * 500 / 2048, 1000, 2000);
  int raw_pitch = map(readSmooth(PIN_JOY_PIT),  0, 4095, 1000, 2000);
  int raw_roll  = map(readSmooth(PIN_JOY_ROLL), 0, 4095, 1000, 2000);

  smooth_thr  = smooth_thr  * (1.0f - ALPHA_RC) + raw_thr   * ALPHA_RC;
  smooth_yaw  = smooth_yaw  * (1.0f - ALPHA_RC) + raw_yaw   * ALPHA_RC;
  smooth_pit  = smooth_pit  * (1.0f - ALPHA_RC) + raw_pitch * ALPHA_RC;
  smooth_roll = smooth_roll * (1.0f - ALPHA_RC) + raw_roll  * ALPHA_RC;

  int val_thr   = (int)smooth_thr;
  int val_yaw   = (int)smooth_yaw;
  int val_pitch = (int)smooth_pit;
  int val_roll  = (int)smooth_roll;

  // ── 2. DEADZONE ───────────────────────────────────────────
  if (abs(val_yaw   - 1500) < YAW_DEADZONE_US) val_yaw   = 1500;  // Yaw deadzone lon 80us
  if (abs(val_pitch - 1500) < 50) val_pitch = 1500;
  if (abs(val_roll  - 1500) < 50) val_roll  = 1500;
  if (val_thr < 1050) val_thr = 1000;
  if (val_thr > 1950) val_thr = 2000;

  // ── 3. NÚT BẤM ───────────────────────────────────────────
  bool btn = digitalRead(PIN_BTN_LOCK);

  if (btn == LOW && lastBtn == HIGH) {
    btnPressStart = now;
    holdFired = false;
  }

  if (btn == LOW && !holdFired && (now - btnPressStart >= HOLD_MS)) {
    holdFired = true;

    if (isLocked) {
      if (!droneDisplay) {
        lcd.clear();
        lcdPrintf(0, 0, " CHUA KET NOI!");
        lcdPrintf(0, 1, "Cho drone bat..");
        armMsgUntil = now + 1200;
        needClear = false;
      } else if (val_thr < 1100) {
        isLocked      = false;
        pendingArm    = true;          // [FIX] Bat dau cho drone xac nhan
        armPendingStart = now;
        lcd.clear();
        lcdPrintf(0, 0, "  * ARMING... *");
        lcdPrintf(0, 1, " Cho drone ARM ");
        armMsgUntil = now + ARM_CONFIRM_TIMEOUT_MS; // Giu man hinh ARMING
        needClear = false;
        Serial.println("[ARM] pendingArm=true - cho fb.isReady tu drone");
      } else {
        lcd.clear();
        lcdPrintf(0, 0, "  GA VE 0 DA!");
        lcdPrintf(0, 1, "  THR: %4d", val_thr);
        armMsgUntil = now + 800;
        needClear = false;
      }
    } else {
      isLocked   = true;
      pendingArm = false;            // [FIX] Huy pending khi LOCK
      lcd.clear();
      lcdPrintf(0, 0, "  -- LOCKED --");
      lcdPrintf(0, 1, "                ");
      armMsgUntil = now + 800;
      needClear = false;
      Serial.println("[LOCK] LOCKED");
    }
  }

  // Nhả nút ngắn → chuyển trang
  if (btn == HIGH && lastBtn == LOW) {
    unsigned long dur = now - btnPressStart;
    if (!holdFired && dur < HOLD_MS) {
      displayPage = (displayPage + 1) % TOTAL_PAGES;
      lcd.clear(); needClear = false;
    }
    holdFired = false;
  }
  lastBtn = btn;

  // ── 4. ĐÓNG GÓI VÀ GỬI LỆNH ESP-NOW (50Hz) ──────────────
  // [FIX] pendingArm: kiem tra xac nhan ARM tu drone
  if (pendingArm && !isLocked) {
    if (fb.isReady) {
      // Drone xac nhan ARM thanh cong!
      pendingArm = false;
      armMsgUntil = now + 1500;
      lcd.clear();
      lcdPrintf(0, 0, " ** READY! **  ");
      lcdPrintf(0, 1, "  Ga len bay!  ");
      needClear = false;
      Serial.println("[ARM] fb.isReady=true - ARMED CONFIRMED!");
    } else if (now - armPendingStart > ARM_CONFIRM_TIMEOUT_MS) {
      // Timeout: drone khong ARM duoc → auto-relock
      pendingArm = false;
      isLocked   = true;
      armMsgUntil = now + 2000;
      lcd.clear();
      lcdPrintf(0, 0, " ARM THAT BAI! ");
      lcdPrintf(0, 1, "Giu nut lai...  ");
      needClear = false;
      Serial.println("[ARM] TIMEOUT - drone khong ARM - auto relock!");
    }
  }

  cmd.lock = isLocked;
  if (isLocked) {
    // LOCKED: gui throttle=1000, sticks center
    cmd.throttle = 1000;
    cmd.lx = 1500; cmd.ly = 1500;
    cmd.rx = 1500; cmd.ry = 1500;
  } else if (pendingArm) {
    // [FIX] PENDING ARM: van gui throttle=1000 de drone du dieu kien ARM
    // Neu gui val_thr ngay, user co the day ga truoc khi drone ARM xong
    cmd.throttle = 1000;
    cmd.lx = 1500; cmd.ly = 1500;
    cmd.rx = 1500; cmd.ry = 1500;
  } else {
    // DA CONFIRMED: gui throttle that
    cmd.throttle = val_thr;
    cmd.lx = val_yaw;
    cmd.ly = 1500;
    cmd.rx = val_roll;
    cmd.ry = val_pitch;
  }

  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    esp_now_send(droneMAC, (uint8_t*)&cmd, sizeof(cmd));
    lastSendTime = now;
  }

  // ── 5. FAILSAFE: TIMEOUT DRONE ───────────────────────────
  if (droneConnected && (now - lastDroneRecvTime > DRONE_TIMEOUT_MS)) {
    droneConnected    = false;
    droneDisconnectAt = now;
    Serial.println("[INFO] Mat telemetry - drone tu xu ly");
  }

  if (droneDisplay && !droneConnected) {
    if (now - droneDisconnectAt > DRONE_DISPLAY_DELAY) {
      droneDisplay = false;
      displayPage  = 0;
      lcd.clear(); needClear = false;
      Serial.println("[LCD] Drone mat ket noi");
    }
  }

  // ── 6. WIFI + MQTT (luôn chạy, không phụ thuộc ARM/LOCK) ──
  handleWiFi(now);
  handleMQTT(now);

  // ── 7. GỬI TELEMETRY MQTT REALTIME ───────────────────────
  // Gửi mỗi 500ms khi có drone, bất kể LOCKED hay ARMED
  if (droneConnected && (now - lastMQTTSend >= 500)) {
    lastMQTTSend = now;

    if (mqttClient.connected()) {
      // Tạo JSON telemetry
      char payload[200];
      snprintf(payload, sizeof(payload),
        "{\"alt\":%.2f,\"p\":%d,\"r\":%d,\"m1\":%d,\"m2\":%d,\"m3\":%d,\"m4\":%d}",
        fb.altitude,
        (int)fb.pitch_val,
        (int)fb.roll_val,
        fb.m1, fb.m2, fb.m3, fb.m4
      );
      bool ok = mqttClient.publish(mqtt_topic, payload);
      Serial.printf("[MQTT] PUB %s -> %s\n", ok ? "OK" : "FAIL", payload);
    } else {
      Serial.println("[MQTT] Chua connected - bo qua");
    }
  }

  // ── 8. LCD (200ms) ────────────────────────────────────────
  static unsigned long lastLCD = 0;
  if (needClear) { lcd.clear(); needClear = false; }
  if (now - lastLCD < 200) { delay(5); return; }
  lastLCD = now;
  if (now < armMsgUntil) return;
  if (armMsgUntil > 0 && now > armMsgUntil) {
    armMsgUntil = 0;
    lcd.clear();
  }
  updateLCD(val_thr, now);
}

// ─────────────────────────────────────────────────────────────
// UI LCD
// ─────────────────────────────────────────────────────────────
void updateLCD(int thr, unsigned long now) {
  if (!droneDisplay) {
    static bool blinkOn = false;
    blinkOn = !blinkOn;
    lcdPrintf(0, 0, blinkOn ? ">> TIM DRONE << " : "                ");
    lcdPrintf(0, 1, "Nhan giu = ARM  ");
    return;
  }

  bool wfOK = wifiOK;
  bool mqOK = mqttClient.connected();

  switch (displayPage) {

    case 0:  // ── STATUS ──────────────────────────────
      lcdPrintf(0, 0, "%-6s T:%-4d %s%s  ",
        isLocked ? "LOCKED" : "ARMED!",
        thr,
        wfOK ? "W" : "w",
        mqOK ? "M" : "m");
      lcdPrintf(0, 1, "DRONE:%-3s SAT:%2d  ",
        fb.isReady ? "RDY" : "LCK",
        fb.sats);
      break;

    case 1:  // ── ATTITUDE ────────────────────────────
      lcdPrintf(0, 0, "P:%-5d R:%-5d  ", fb.pitch_val, fb.roll_val);
      lcdPrintf(0, 1, "HDG:%-4d MAG:%-2s  ",
        fb.heading,
        fb.mag ? "OK" : "--");
      break;

    case 2:  // ── ALTITUDE ────────────────────────────
      lcdPrintf(0, 0, "ALT: %.2fm        ", fb.altitude);
      lcdPrintf(0, 1, "GPS:%-2s  BAR:%-2s   ",
        fb.gps  ? "OK" : "NO",
        fb.baro ? "OK" : "ER");
      break;

    case 3:  // ── MOTOR PWM ───────────────────────────
      lcdPrintf(0, 0, "M1:%4d M2:%4d   ", fb.m1, fb.m2);
      lcdPrintf(0, 1, "M3:%4d M4:%4d   ", fb.m3, fb.m4);
      break;

    case 4:  // ── SENSOR STATUS ─────────────────────────
      lcdPrintf(0, 0, "MPU:%-2s BRO:%-2s    ",
        fb.mpu  ? "OK" : "ER",
        fb.baro ? "OK" : "ER");
      lcdPrintf(0, 1, "GPS:%-2s MAG:%-2s    ",
        fb.gps ? "OK" : "--",
        fb.mag ? "OK" : "--");
      break;

    case 5: {  // ── NETWORK STATUS ─────────────────────
      if (wfOK) {
        lcdPrintf(0, 0, "WiFi: OK        ");
      } else {
        lcdPrintf(0, 0, "WiFi: OFF wait..");
      }
      lcdPrintf(0, 1, "MQ:%-2s  ESP:%-4s  ",
        mqOK         ? "OK" : "--",
        droneDisplay ? "CONN" : "LOST");
      break;
    }

    default:
      displayPage = 0;
      break;
  }
}
