/*
 * ============================================================
 *  DIAGNOSE_MPU — Chẩn đoán MPU6050 lỗi intermittent
 *
 *  Upload sketch này lên ESP32 (thay drone_main.ino tạm thời)
 *  Mở Serial Monitor @ 115200 baud
 *  Quan sát kết quả, chụp ảnh/copy log gửi lại
 * ============================================================
 */
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

// ─── Đọc 1 byte từ register của MPU ───────────────────────
uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  int err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.printf("  [ERR] endTransmission err=%d\n", err);
    return 0xFF;
  }
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

// ─── Ghi 1 byte vào register ──────────────────────────────
uint8_t writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission();
}

// ─── I2C Bus Recovery (9 SCL pulses) ─────────────────────
void i2cBusRecovery() {
  Serial.println("  [BUS] Bus recovery: 9 SCL pulses...");
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    if (digitalRead(SDA_PIN) == HIGH) {
      Serial.printf("  [BUS] SDA freed after %d pulses\n", i + 1);
      break;
    }
  }
  // STOP condition
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(10);
}

// ─── Scan toàn bộ I2C bus ─────────────────────────────────
void scanI2C() {
  Serial.println("\n[SCAN] Quét I2C bus...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  [FOUND] 0x%02X", addr);
      if (addr == 0x68 || addr == 0x69) Serial.print(" ← MPU6050");
      if (addr == 0x1E)                  Serial.print(" ← HMC5883L (la ban)");
      if (addr == 0x76 || addr == 0x77) Serial.print(" ← BMP280/BMP180");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("  [WARN] Không tìm thấy thiết bị nào!");
  Serial.printf("[SCAN] Tổng: %d thiết bị\n", found);
}

// ─── Kiểm tra chi tiết MPU6050 ────────────────────────────
void diagMPU() {
  Serial.println("\n[MPU] === Kiểm tra chi tiết MPU6050 ===");

  // Bước 1: Ping
  Wire.beginTransmission(MPU_ADDR);
  int pingErr = Wire.endTransmission();
  Serial.printf("  Ping 0x68: err=%d (%s)\n", pingErr, pingErr == 0 ? "OK" : "FAIL-NACK");
  if (pingErr != 0) {
    Serial.println("  → Chip không respond! Kiểm tra dây hoặc địa chỉ.");
    return;
  }

  // Bước 2: WHO_AM_I (register 0x75) — phải = 0x68
  uint8_t whoami = readReg(MPU_ADDR, 0x75);
  Serial.printf("  WHO_AM_I (0x75) = 0x%02X (mong đợi: 0x68)\n", whoami);
  if (whoami != 0x68) {
    Serial.println("  → WHO_AM_I sai! Có thể là MPU6500/ICM chip khác.");
  }

  // Bước 3: PWR_MGMT_1 (0x6B) — bit 6 = SLEEP
  uint8_t pwr = readReg(MPU_ADDR, 0x6B);
  Serial.printf("  PWR_MGMT_1 (0x6B) = 0x%02X | SLEEP=%d RESET=%d\n",
                pwr, (pwr >> 6) & 1, (pwr >> 7) & 1);
  if ((pwr >> 6) & 1) {
    Serial.println("  → Chip đang SLEEP! Sẽ wake up...");
    writeReg(MPU_ADDR, 0x6B, 0x00);
    delay(100);
    pwr = readReg(MPU_ADDR, 0x6B);
    Serial.printf("  PWR_MGMT_1 sau wake-up = 0x%02X\n", pwr);
  }
  if ((pwr >> 7) & 1) {
    Serial.println("  → RESET bit vẫn set! Chip chưa hoàn tất reset.");
  }

  // Bước 4: INT_STATUS (0x3A)
  uint8_t intSt = readReg(MPU_ADDR, 0x3A);
  Serial.printf("  INT_STATUS (0x3A) = 0x%02X\n", intSt);

  // Bước 5: CONFIG (0x1A) — DLPF
  uint8_t cfg = readReg(MPU_ADDR, 0x1A);
  Serial.printf("  CONFIG/DLPF (0x1A) = 0x%02X\n", cfg);

  // Bước 6: GYRO_CONFIG (0x1B)
  uint8_t gcfg = readReg(MPU_ADDR, 0x1B);
  Serial.printf("  GYRO_CONFIG (0x1B) = 0x%02X (FS_SEL=%d)\n", gcfg, (gcfg >> 3) & 3);

  // Bước 7: Đọc thử dữ liệu Accel raw
  uint8_t axH = readReg(MPU_ADDR, 0x3B);
  uint8_t axL = readReg(MPU_ADDR, 0x3C);
  int16_t ax = (int16_t)((axH << 8) | axL);
  uint8_t ayH = readReg(MPU_ADDR, 0x3D);
  uint8_t ayL = readReg(MPU_ADDR, 0x3E);
  int16_t ay = (int16_t)((ayH << 8) | ayL);
  uint8_t azH = readReg(MPU_ADDR, 0x3F);
  uint8_t azL = readReg(MPU_ADDR, 0x40);
  int16_t az = (int16_t)((azH << 8) | azL);
  Serial.printf("  Accel raw: X=%d Y=%d Z=%d\n", ax, ay, az);
  if (ax == 0 && ay == 0 && az == 0) {
    Serial.println("  → Tất cả 0! Chip không đọc được hoặc vẫn SLEEP.");
  } else {
    Serial.println("  → Accel data OK!");
  }

  // Bước 8: INT_PIN_CFG (0x37) — bypass mode
  uint8_t bypass = readReg(MPU_ADDR, 0x37);
  Serial.printf("  INT_PIN_CFG (0x37) = 0x%02X | I2C_BYPASS=%d\n",
                bypass, (bypass >> 1) & 1);
  if ((bypass >> 1) & 1) {
    Serial.println("  → Bypass ON: HMC5883L kết nối thẳng bus chính.");
    Serial.println("    Nếu HMC5883L bị treo → toàn bộ bus treo!");
  }

  Serial.println("[MPU] === Kết thúc chẩn đoán ===");
}

// ─── Stress test: lặp 20 lần để kiểm tra intermittent ────
void stressTest() {
  Serial.println("\n[STRESS] Lặp 20 lần ping MPU...");
  int ok = 0, fail = 0;
  for (int i = 0; i < 20; i++) {
    Wire.beginTransmission(MPU_ADDR);
    int e = Wire.endTransmission();
    if (e == 0) { ok++; Serial.print("."); }
    else         { fail++; Serial.print("X"); }
    delay(50);
  }
  Serial.printf("\n  OK=%d FAIL=%d → ", ok, fail);
  if (fail == 0)      Serial.println("Bus ổn định ✓");
  else if (fail < 5)  Serial.println("Bus thỉnh thoảng lỗi → Nguồn điện yếu / dây xấu");
  else                Serial.println("Bus lỗi nhiều → Kết nối vật lý có vấn đề!");
}

// ─── Kiểm tra SDA/SCL bị stuck ───────────────────────────
void checkBusLines() {
  Serial.println("\n[BUS] Kiểm tra trạng thái SDA/SCL...");
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(5);
  int sda = digitalRead(SDA_PIN);
  int scl = digitalRead(SCL_PIN);
  Serial.printf("  SDA=%d SCL=%d\n", sda, scl);
  if (sda == LOW) Serial.println("  ⚠ SDA stuck LOW! Bus bị treo. Cần bus recovery.");
  if (scl == LOW) Serial.println("  ⚠ SCL stuck LOW! Clock bị treo — hiếm gặp, có thể chip lỗi.");
  if (sda == HIGH && scl == HIGH) Serial.println("  → Bus lines OK (HIGH = idle)");
  // Restore I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
}

// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println(" DIAGNOSE MPU — GY-87 / MPU6050");
  Serial.println("========================================");

  // Bước 0: Kiểm tra dây SDA/SCL
  checkBusLines();

  // Nếu SDA stuck, thử recovery
  pinMode(SDA_PIN, INPUT_PULLUP);
  if (digitalRead(SDA_PIN) == LOW) {
    i2cBusRecovery();
  } else {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    delay(100);
  }

  // Bước 1: Quét toàn bus
  scanI2C();

  // Bước 2: Chẩn đoán MPU chi tiết
  diagMPU();

  // Bước 3: Stress test
  stressTest();

  Serial.println("\n[DONE] Gửi log này để được hỗ trợ.");
  Serial.println("  Cắm pin lại và bấm RESET để chạy lại.");
}

void loop() {
  // In liên tục Accel để xem có mất liên kết không
  static unsigned long last = 0;
  if (millis() - last > 1000) {
    last = millis();
    uint8_t axH = readReg(MPU_ADDR, 0x3B);
    uint8_t axL = readReg(MPU_ADDR, 0x3C);
    int16_t ax  = (int16_t)((axH << 8) | axL);
    uint8_t azH = readReg(MPU_ADDR, 0x3F);
    uint8_t azL = readReg(MPU_ADDR, 0x40);
    int16_t az  = (int16_t)((azH << 8) | azL);
    
    Wire.beginTransmission(MPU_ADDR);
    int e = Wire.endTransmission();
    Serial.printf("[LIVE] Ping=%s | AX=%d AZ=%d\n",
                  e == 0 ? "OK" : "FAIL", ax, az);
  }
}
