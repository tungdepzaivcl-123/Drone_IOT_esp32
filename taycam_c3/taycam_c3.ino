/*
 * ================================================================
 *  TAY CAM DRONE V3.0 — ESP32-C3
 *  Dựa trên kết quả test thực tế (tayxin_test.ino):
 *
 *  JOYSTICK (đã xác nhận đúng chiều):
 *    GPIO 0  → Joystick Trái Y  (Throttle — Lên/Xuống)
 *    GPIO 1  → Joystick Trái X  (Yaw — Trái/Phải)
 *    GPIO 2  → Joystick Phải X  (Roll — Trái/Phải)
 *    GPIO 3  → Joystick Phải Y  (Pitch — Lên/Xuống, đảo ngược)
 *
 *  NÚT BẤM (chỉ 1 nút hoạt động):
 *    GPIO 9  → BTN_ARM: Bấm 1 lần = ARM, bấm lần nữa = DISARM
 *
 *  OLED:
 *    GPIO 5  → SDA
 *    GPIO 6  → SCL
 *
 *  DRONE MAC: Để {0xFF,...} → tay cầm TỰ ĐỘNG bắt MAC từ telemetry
 * ================================================================
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
// DRONE MAC — để 0xFF sẽ tự động học từ telemetry
// ================================================================
uint8_t DRONE_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#define ESPNOW_CHANNEL 1

// ================================================================
// STRUCT — khớp 100% với drone_main.ino
// ================================================================
typedef struct __attribute__((packed)) {
  int  lx, ly, rx, ry;
  bool lock;
  int  throttle;
} struct_command;

typedef struct __attribute__((packed)) {
  bool    mpu;
  bool    baro;
  bool    gps;
  bool    mag;
  int16_t pitch_val;
  int16_t roll_val;
  float   altitude;
  int     sats;
  bool    isReady;
  int     m1, m2, m3, m4;
  int     heading;
} struct_status;

// ================================================================
// CHÂN PIN (đã xác nhận bằng test)
// ================================================================
#define JOY_L_Y   0   // Throttle (lên/xuống tay trái)
#define JOY_L_X   1   // Yaw      (trái/phải tay trái)
#define JOY_R_X   2   // Roll     (trái/phải tay phải)
#define JOY_R_Y   3   // Pitch    (lên/xuống tay phải)

#define BTN_ARM   9   // Nút duy nhất: toggle ARM/DISARM

#define I2C_SDA   5
#define I2C_SCL   6

// ================================================================
// OLED
// ================================================================
#define OLED_W    128
#define OLED_H     64
#define OLED_ADDR  0x3C
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
bool oledOK = false;

// ================================================================
// JOYSTICK CALIBRATION (từ kết quả test)
// ================================================================
#define JOY_LOW       80
#define JOY_CENTER  2048
#define JOY_HIGH    4010
#define JOY_DEADZONE 200   // Vùng chết đã test

// Chiều đúng (đã xác nhận):
#define INV_YAW    false   // Joystick trái X → Yaw
#define INV_THR    false   // Joystick trái Y → Throttle
#define INV_ROLL   false   // Joystick phải X → Roll
#define INV_PITCH  false   // [V5.3 FIX] Đổi true→false: đẩy lên=tiến, kéo về=lùi

// Throttle: xuống = 1000, lên = 2000
// Ngưỡng ga thấp để ARM (kéo cần xuống dưới mức này mới ARM được)
#define THR_ARM_THRESHOLD 1100

// ================================================================
// STATE
// ================================================================
bool isArmed = false;           // false=LOCKED, true=ARMED

struct_command cmd;
struct_status  fb;
bool           fbUpdated  = false;
bool           espNowOK   = false;
unsigned long  lastRecvMs = 0;
unsigned long  lastSendMs = 0;
#define SEND_INTERVAL_MS  20    // 50Hz
#define LINK_TIMEOUT_MS 1500

// Debounce nút GPIO9
bool          prevBtn     = false;
unsigned long btnPressMs  = 0;
bool          btnHandled  = false;
#define BTN_DEBOUNCE_MS  50

// ================================================================
// MAP JOYSTICK
// ================================================================
int mapJoystick(int raw, bool invert) {
  int c = raw - JOY_CENTER;
  if (abs(c) < JOY_DEADZONE) c = 0;
  int us;
  if (c >= 0) {
    us = 1500 + (int)((long)c * 500 / (JOY_HIGH - JOY_CENTER));
  } else {
    us = 1500 + (int)((long)c * 500 / (JOY_CENTER - JOY_LOW));
  }
  us = constrain(us, 1000, 2000);
  return invert ? (3000 - us) : us;
}

int mapThrottle(int raw, bool invert) {
  int us = (int)map((long)raw, JOY_LOW, JOY_HIGH, 1000, 2000);
  us = constrain(us, 1000, 2000);
  return invert ? (3000 - us) : us;
}

// ================================================================
// ESP-NOW CALLBACKS
// ================================================================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(struct_status)) return;
  memcpy(&fb, data, sizeof(fb));
  fbUpdated  = true;
  lastRecvMs = millis();

  // TỰ ĐỘNG HỌC MAC DRONE từ gói telemetry đầu tiên
  if (DRONE_MAC[0] == 0xFF) {
    memcpy(DRONE_MAC, info->src_addr, 6);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, DRONE_MAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
      espNowOK = true;
      Serial.printf("\n[AUTO-MAC] Bat duoc MAC drone: %02X:%02X:%02X:%02X:%02X:%02X\n\n",
        DRONE_MAC[0], DRONE_MAC[1], DRONE_MAC[2],
        DRONE_MAC[3], DRONE_MAC[4], DRONE_MAC[5]);
    }
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t st) {
  (void)info; (void)st;
}

// ================================================================
// OLED
// ================================================================
void updateOLED() {
  if (!oledOK) return;
  bool linked = (lastRecvMs > 0) && (millis() - lastRecvMs < LINK_TIMEOUT_MS);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Dòng 1: Trạng thái ARM + LINK
  oled.setCursor(0, 0);
  if (isArmed) {
    oled.print("[** ARMED **]");
  } else {
    oled.print("[ DISARMED  ]");
  }
  oled.setCursor(84, 0);
  oled.print(linked ? "LINK" : "----");

  oled.drawLine(0, 9, OLED_W, 9, SSD1306_WHITE);

  // Dòng 2: Throttle bar
  oled.setCursor(0, 11);
  oled.printf("THR%4d", cmd.throttle);
  int barW = map(cmd.throttle, 1000, 2000, 0, 78);
  barW = constrain(barW, 0, 78);
  oled.drawRect(49, 11, 78, 7, SSD1306_WHITE);
  if (barW > 0) oled.fillRect(49, 11, barW, 7, SSD1306_WHITE);

  // Dòng 3: Roll / Pitch / Yaw
  oled.setCursor(0, 21);
  oled.printf("R%+4d P%+4d Y%+4d",
    cmd.rx - 1500, cmd.ry - 1500, cmd.lx - 1500);

  oled.drawLine(0, 31, OLED_W, 31, SSD1306_WHITE);

  // Dòng 4-6: Telemetry từ drone
  oled.setCursor(0, 33);
  if (linked) {
    oled.printf("ALT%5.1fm", fb.altitude);
    oled.setCursor(0, 43);
    oled.printf("M1%4d M2%4d", fb.m1, fb.m2);
    oled.setCursor(0, 53);
    oled.printf("M3%4d M4%4d", fb.m3, fb.m4);
  } else {
    oled.setCursor(0, 38);
    if (espNowOK) {
      oled.println("Cho drone...");
    } else {
      oled.println("Tim drone...");
      oled.setCursor(0, 50);
      oled.printf("MAC?");
    }
  }

  oled.display();
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] Tay Cam V3.0 — ESP32-C3");
  Serial.println("[BOOT] 1 nut GPIO9: bam = ARM/DISARM toggle");

  // Nút
  pinMode(BTN_ARM, INPUT_PULLUP);

  // ADC
  analogSetAttenuation(ADC_11db);

  // OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0,  0); oled.println("TAY CAM V3.0");
    oled.setCursor(0, 12); oled.println("ESP32-C3");
    oled.setCursor(0, 24); oled.println("Khoi dong ESP-NOW...");
    oled.display();
  }

  // WiFi STA cho ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setChannel(ESPNOW_CHANNEL);
  delay(100);
  Serial.printf("[WIFI] MAC: %s | CH: %d\n", WiFi.macAddress().c_str(), ESPNOW_CHANNEL);

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW INIT THAT BAI!");
    if (oledOK) {
      oled.clearDisplay();
      oled.setCursor(0, 10); oled.println("ESP-NOW FAIL!");
      oled.display();
    }
    while (1) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Khởi tạo cmd an toàn
  cmd.lx       = 1500;
  cmd.ly       = 1000;
  cmd.rx       = 1500;
  cmd.ry       = 1500;
  cmd.lock     = true;
  cmd.throttle = 1000;
  isArmed      = false;

  if (oledOK) {
    oled.clearDisplay();
    oled.setCursor(0,  0); oled.println("DISARMED");
    oled.setCursor(0, 14); oled.println("Cho drone phat song...");
    oled.setCursor(0, 28); oled.println("Bam GPIO9 de ARM");
    oled.display();
    delay(1000);
  }

  Serial.println("[OK] San sang! Bam GPIO9 de ARM, bam lai de DISARM");
  lastSendMs = millis();
}

// ================================================================
// LOOP — 50Hz
// ================================================================
void loop() {
  unsigned long now = millis();

  // ── 1. Đọc joystick ────────────────────────────────
  int raw_lx = analogRead(JOY_L_X);  // GPIO1 — Yaw
  int raw_ly = analogRead(JOY_L_Y);  // GPIO0 — Throttle
  int raw_rx = analogRead(JOY_R_X);  // GPIO2 — Roll
  int raw_ry = analogRead(JOY_R_Y);  // GPIO3 — Pitch

  int joyLX  = mapJoystick(raw_lx, INV_YAW);
  int joyLY  = mapThrottle(raw_ly, INV_THR);  // Throttle: map thẳng
  int joyRX  = mapJoystick(raw_rx, INV_ROLL);
  int joyRY  = mapJoystick(raw_ry, INV_PITCH);

  // ── 2. Đọc nút GPIO9 (debounce) ────────────────────
  bool btn = !digitalRead(BTN_ARM);  // LOW khi bấm (PULLUP)

  // Phát hiện cạnh lên (nhả ra)
  if (btn && !prevBtn) {
    btnPressMs = now;
    btnHandled = false;
  }
  if (!btn && prevBtn && !btnHandled) {
    // Nhả ra sau debounce → toggle ARM/DISARM
    if (now - btnPressMs >= BTN_DEBOUNCE_MS) {
      btnHandled = true;
      if (!isArmed) {
        // Muốn ARM: kiểm tra ga phải thấp
        if (joyLY < THR_ARM_THRESHOLD) {
          isArmed = true;
          Serial.println("[RC] --> ARMED! Bay duoc!");
        } else {
          Serial.printf("[RC] ARM TU CHOI: ga qua cao! joyLY=%d (can < %d)\n",
            joyLY, THR_ARM_THRESHOLD);
          // Nhấp OLED cảnh báo
          if (oledOK) {
            oled.clearDisplay();
            oled.setCursor(0, 20); oled.setTextSize(2);
            oled.println("GA CAO!");
            oled.setTextSize(1);
            oled.setCursor(0, 42); oled.println("Keo ga xuong roi ARM");
            oled.display();
          }
        }
      } else {
        // DISARM
        isArmed = false;
        Serial.println("[RC] --> DISARMED!");
      }
    }
  }
  prevBtn = btn;

  // ── 3. Build gói lệnh ──────────────────────────────
  if (isArmed) {
    cmd.lock     = false;
    cmd.throttle = joyLY;
    cmd.lx       = joyLX;
    cmd.ly       = joyLY;
    cmd.rx       = joyRX;
    cmd.ry       = joyRY;
  } else {
    // DISARMED: lock=true, tất cả về safe
    cmd.lock     = true;
    cmd.throttle = 1000;
    cmd.lx       = 1500;
    cmd.ly       = 1000;
    cmd.rx       = 1500;
    cmd.ry       = 1500;
  }

  // ── 4. Gửi ESP-NOW 50Hz (chỉ khi biết MAC drone) ──
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    if (espNowOK) {
      esp_now_send(DRONE_MAC, (uint8_t*)&cmd, sizeof(cmd));
    }
  }

  // ── 5. Cập nhật OLED 100ms ─────────────────────────
  static unsigned long lastOLED = 0;
  if (now - lastOLED >= 100) {
    lastOLED = now;
    updateOLED();
  }

  // ── 6. Serial debug 1s ─────────────────────────────
  static unsigned long lastDbg = 0;
  if (now - lastDbg >= 1000) {
    lastDbg = now;
    bool linked = (lastRecvMs > 0) && (now - lastRecvMs < LINK_TIMEOUT_MS);
    Serial.printf("[RC] %s | lock=%d thr=%d rx=%d ry=%d lx=%d | joyLY=%d | Link=%s\n",
      isArmed ? "ARMED " : "DISARM",
      cmd.lock, cmd.throttle, cmd.rx, cmd.ry, cmd.lx, joyLY,
      linked ? "OK" : "NO");
    if (linked) {
      Serial.printf("     Drone: ARMED=%d ALT=%.1fm M1=%d M2=%d M3=%d M4=%d\n",
        fb.isReady, fb.altitude, fb.m1, fb.m2, fb.m3, fb.m4);
    }
  }

  delay(5); // ~200Hz max, ESP-NOW gửi 50Hz
}
