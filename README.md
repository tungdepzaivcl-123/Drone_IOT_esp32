# 🚁 Drone IoT ESP32 — QAV250 Flight Controller

> **Tự chế Flight Controller cho QAV250 dùng ESP32 + MPU6050**  
> Điều khiển qua ESP-NOW (không cần WiFi router) | Web UI PID tuning

[![ESP32](https://img.shields.io/badge/MCU-ESP32-blue)](https://www.espressif.com/)
[![Arduino](https://img.shields.io/badge/Framework-Arduino-teal)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## 📋 Mục lục

- [Tổng quan](#-tổng-quan)
- [Phần cứng](#-phần-cứng)
- [Sơ đồ kết nối](#-sơ-đồ-kết-nối)
- [Cấu trúc dự án](#-cấu-trúc-dự-án)
- [Cài đặt môi trường](#-cài-đặt-môi-trường)
- [Flash firmware](#-flash-firmware)
- [Web UI](#-web-ui)
- [PID Tuning](#-pid-tuning)
- [Motor Layout](#-motor-layout)
- [Công cụ test](#-công-cụ-test)
- [Thông số kỹ thuật](#-thông-số-kỹ-thuật)
- [Troubleshooting](#-troubleshooting)

---

## 🎯 Tổng quan

Dự án xây dựng **Flight Controller tự chế** hoàn chỉnh cho drone Quadcopter QAV250:

```
┌─────────────────────────────────────────────────────┐
│                  HỆ THỐNG DRONE                      │
│                                                     │
│  [Tay cầm ESP32]  ──ESP-NOW──►  [Drone ESP32]       │
│   taycamxin.ino                  drone_main.ino     │
│                                        │            │
│                              ┌─────────┴────────┐   │
│                              │   MPU6050 (IMU)  │   │
│                              │   BMP280 (Baro)  │   │
│                              │   4x Motor ESC   │   │
│                              └──────────────────┘   │
│                                                     │
│  [Web Browser]  ──WiFi──►  http://192.168.4.1       │
│   PID Tuning UI              Config WebServer       │
└─────────────────────────────────────────────────────┘
```

### Tính năng chính:
- ✅ **Angle Mode** — Cascade PID (Outer angle + Inner rate)
- ✅ **ESP-NOW** — Truyền dữ liệu không dây, độ trễ < 5ms
- ✅ **Web UI** — Chỉnh PID qua browser, không cần IDE
- ✅ **Motor Test** — Test từng motor riêng qua Web
- ✅ **MPU Vibration Test** — Đo nhiễu rung động tự động
- ✅ **Alt-Hold** — Giữ độ cao bằng BMP280 (thực nghiệm)
- ✅ **Failsafe** — Tự disarm khi mất tín hiệu > 4 giây
- ✅ **Anti-windup** — Giới hạn I-term tích lũy

---

## 🔧 Phần cứng

| Linh kiện | Model | Ghi chú |
|-----------|-------|---------|
| MCU Drone | ESP32 WROOM-32 | Flight Controller |
| MCU Tay cầm | ESP32 C3 / WROOM | Remote Controller |
| IMU | MPU6050 (GY-87) | SDA=21, SCL=22 |
| Barometer | BMP280 | I2C 0x76 |
| Frame | QAV250 Carbon Fiber | 250mm wheelbase |
| Motor | MT2204 2300KV | 4x (2 CW + 2 CCW) |
| Propeller | 5045 3-blade | 5 inch |
| ESC | 20A (Digital PWM) | 250Hz PWM |
| Pin | LiPo 3S/4S | 11.1V / 14.8V |

---

## 📌 Sơ đồ kết nối

### ESP32 → ESC (Motor)
```
GPIO 27 ──► ESC M1 (Front-Right,  CCW) ──► Motor FR
GPIO 26 ──► ESC M2 (Front-Left,   CW)  ──► Motor FL
GPIO 14 ──► ESC M3 (Rear-Right,   CW)  ──► Motor RR
GPIO 13 ──► ESC M4 (Rear-Left,    CCW) ──► Motor RL
```

### ESP32 → MPU6050 / BMP280 (I2C)
```
GPIO 21 (SDA) ──► SDA
GPIO 22 (SCL) ──► SCL
3.3V          ──► VCC
GND           ──► GND
```

### Motor Layout (Quad-X, nhìn từ trên)
```
       [TRƯỚC / FRONT]
            ▲
  FL(M2,CW) │ FR(M1,CCW)
     ↻      │      ↺
     ────────●────────
     ↺      │      ↻
  RL(M4,CCW)│ RR(M3,CW)
            │
       [SAU / REAR]

  ↻ = CW  (Dây thẳng)
  ↺ = CCW (Dây chéo)
```

---

## 📁 Cấu trúc dự án

```
Drone_IOT_esp32/
│
├── 📂 drone_main/              # FLIGHT CONTROLLER (DRONE)
│   ├── drone_main.ino          # ← Code chính để flash vào ESP32 drone
│   └── diagnose_mpu.ino        # Tool chẩn đoán MPU6050
│
├── 📂 drone_rc/                # REMOTE CONTROLLER (TAY CẦM)
│   └── drone_rc.ino            # Code tay cầm (ESP32)
│
├── 📂 taycamxin/               # TAY CẦM PHIÊN BẢN CŨ
│   └── taycamxin.ino           # Code tay cầm với LCD
│
├── 📂 taycam_c3/               # TAY CẦM ESP32-C3
│   └── taycam_c3.ino           # Code tay cầm nhỏ gọn
│
├── 📂 motor_test/              # TEST MOTOR ĐƠN LẺ
│   └── motor_test.ino          # ← Flash để test từng motor qua Web
│
├── 📂 mpu_vibration_test/      # TEST NHIỄU RUNG ĐỘNG
│   └── mpu_vibration_test.ino  # ← Flash để đo noise MPU theo throttle
│
├── 📂 xe_canbang/              # DỰ ÁN PHỤ: XE CÂN BẰNG
│   └── ...
│
├── pid_tuner.html              # PID Tuner offline (mở bằng browser)
├── drone_sim.html              # Drone simulator (mở bằng browser)
├── drone_esp32_sim.py          # Mô phỏng PID Python
├── autotune.py                 # Auto-tune PID bằng Ziegler-Nichols
└── README.md                   # File này
```

---

## 💻 Cài đặt môi trường

### 1. Cài Arduino IDE 2.x
Download: https://www.arduino.cc/en/software

### 2. Thêm ESP32 board
`File → Preferences → Additional Boards Manager URLs`:
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
Sau đó: `Tools → Board → Boards Manager → Tìm "esp32" → Install`

### 3. Cài thư viện (Library Manager)
```
Tools → Manage Libraries → Tìm và cài:
  - MPU6050_light    (by rfetick)
  - Adafruit BMP280  (by Adafruit)
  - esp_now          (built-in với ESP32)
```

### 4. Cấu hình Board
```
Tools → Board     → ESP32 Dev Module
Tools → Port      → COMxx (chọn đúng port)
Tools → CPU Freq  → 240MHz
Tools → Flash Size → 4MB
```

---

## 🔥 Flash firmware

### Thứ tự flash (quan trọng!)

```
Bước 1: Flash DRONE (drone_main.ino)
         → Mở Serial Monitor, ghi lại địa chỉ MAC của drone

Bước 2: Điền MAC drone vào taycamxin.ino / drone_rc.ino
         → uint8_t remoteMAC[] = {0x??,...};

Bước 3: Flash TAY CẦM (taycamxin.ino hoặc drone_rc.ino)

Bước 4: Test kết nối ESP-NOW qua Serial Monitor
```

### Địa chỉ MAC trong drone_main.ino
```cpp
// Dòng ~117 trong drone_main.ino — đổi thành MAC của tay cầm bạn
uint8_t remoteMAC[] = {0xB0, 0xA6, 0x04, 0x58, 0x94, 0x48};
```

---

## 🌐 Web UI

Sau khi drone bật nguồn:

1. Kết nối WiFi: **`Drone-Config`** / mật khẩu: **`drone1234`**
2. Mở browser: **`http://192.168.4.1`**

### Các trang có sẵn:

| URL | Chức năng |
|-----|-----------|
| `http://192.168.4.1/` | Trang chính — PID Config |
| `http://192.168.4.1/motortest` | Test motor riêng lẻ |
| `http://192.168.4.1/status` | Trạng thái drone (JSON) |

### Motor Test (WiFi riêng):
- Flash `motor_test/motor_test.ino`
- Kết nối WiFi: **`MotorTest`** / **`motor1234`**
- Mở: `http://192.168.4.1`

---

## 📊 PID Tuning

### Thông số hiện tại (V7.4)
```cpp
//                    Roll    Pitch   Yaw
kp_inner[3]       = { 0.32,   0.45,  1.00 }   // Rate P
ki_inner[3]       = { 0.20,   0.10,  0.30 }   // Rate I
kd_inner[3]       = { 0.025,  0.055, 0.00 }   // Rate D
kp_angle          = 1.50                        // Angle P (outer loop)
alpha_gyro        = 0.70                        // EMA gyro filter
alpha_d           = 0.30                        // EMA D-term filter
DLPF              = 0x04 (21Hz)                 // Hardware low-pass filter
```

### Hướng dẫn tune nhanh:

| Triệu chứng | Giải pháp |
|-------------|-----------|
| Rung nhanh, nhỏ | Giảm `kp_inner` |
| Lắc chậm, không tắt | Giảm `ki_inner` hoặc tăng `kd_inner` |
| Phản ứng chậm, lừ đừ | Tăng `kp_inner` hoặc tăng `kp_angle` |
| Drift 1 hướng liên tục | Chỉnh `pitch_trim` / `roll_trim` (±0.5°) |
| Xoay tròn khi cất cánh | Tăng `kp_inner[2]` (Yaw) |

### Sử dụng Web UI để tune:
1. Mở `http://192.168.4.1`
2. Chỉnh giá trị PID
3. Nhấn **💾 LUU PID VAO DRONE**
4. Bay thử và lặp lại

---

## 🔬 Công cụ test

### 1. Motor Test (`motor_test/`)
```
Mục đích : Xác định motor nào ở vị trí nào (FR/FL/RR/RL)
Flash vào: ESP32 drone (riêng biệt, không cần PIN bay)
WiFi      : MotorTest / motor1234
URL       : http://192.168.4.1
Max pulse : 1450µs (an toàn, không đủ lực nâng)
```
> ⚠️ **THÁO CÁNH QUẠT trước khi test!**

### 2. MPU Vibration Test (`mpu_vibration_test/`)
```
Mục đích  : Đo nhiễu rung động MPU theo từng mức throttle
Flash vào : ESP32 drone (riêng biệt)
Kết quả   : Tự động in báo cáo và khuyến nghị alpha_gyro
Thời gian : ~60 giây (tự động hoàn toàn)
```
Quy trình tự động:
```
Boot → Calibrate MPU (3s) → ESC arm (2s) →
THR=0% → THR=15% → THR=25% → THR=35% → THR=45%
→ In báo cáo tổng hợp + khuyến nghị filter
```
> ⚠️ **THÁO CÁNH QUẠT trước khi test!**

### 3. PID Tuner Offline (`pid_tuner.html`)
Mở file `pid_tuner.html` bằng bất kỳ browser nào để mô phỏng đáp ứng PID mà không cần drone.

### 4. Drone Simulator (`drone_sim.html`)
Mô phỏng vật lý drone 2D trong browser.

---

## ⚙️ Thông số kỹ thuật

| Thông số | Giá trị |
|----------|---------|
| Loop rate | 250 Hz |
| PWM frequency | 250 Hz (12-bit) |
| I2C speed | 400 kHz |
| IMU DLPF | 21 Hz (hardware) |
| EMA gyro alpha | 0.70 |
| Max tilt failsafe | 60° |
| Fast tilt failsafe | 45° + 300°/s |
| Arming timeout | 4–8 giây (adaptive) |
| Rearm cooldown | 2 giây |
| ESP-NOW channel | 1 |

---

## 🛠️ Troubleshooting

### ❌ MPU6050 không tìm thấy
```
→ Kiểm tra dây SDA=21, SCL=22
→ Kiểm tra nguồn 3.3V (KHÔNG dùng 5V)
→ Chạy diagnose_mpu.ino để kiểm tra I2C bus
```

### ❌ Drone xoay tròn khi cất cánh
```
→ Test từng motor bằng motor_test.ino
→ Kiểm tra chiều quay motor đúng layout Quad-X
→ Tăng Yaw P trong Web UI (mặc định 1.00)
→ Kiểm tra cánh quạt đúng chiều (gió phải đẩy XUỐNG)
```

### ❌ Drone lật ra đằng sau
```
→ Đảm bảo drone nằm PHẲNG khi Reset/boot để MPU calibrate đúng
→ Kiểm tra trọng tâm (CG): pin phải ở giữa, không lệch đuôi
→ Chỉnh pitch_trim trong Web UI
```

### ❌ Lướt sóng (porpoising) đầu đuôi
```
→ Giảm alpha_d (0.20 → 0.15)
→ Giảm ki_inner[1] (Pitch I)
→ Tăng kd_inner[1] (Pitch D)
```

### ❌ DLPF không được ghi (vẫn là 0x00)
```
→ setupMPUFilter() PHẢI được gọi SAU mpu.calcOffsets()
→ Xem log Serial: [DLPF] ... OK
→ Thư viện MPU6050_light reset DLPF trong calcOffsets()!
```

### ❌ ESP-NOW không kết nối
```
→ Kiểm tra MAC address trong code
→ Cả 2 ESP32 phải dùng WiFi channel 1
→ Tay cầm dùng WiFi STA mode, drone dùng AP+STA
```

---

## 📱 Sơ đồ ARM / DISARM

```
ARM   : Giữ nút 2 giây + Throttle < 1100µs + Sticks centered
DISARM: Nhấn nút LOCK hoặc nghiêng > 60° hoặc mất tín hiệu > 4s
```

---

## 📈 Lịch sử phiên bản

| Version | Thay đổi |
|---------|----------|
| V7.4 | Fix PID Pitch direction, DLPF 21Hz, vibration test |
| V7.3 | Fix Pitch positive feedback (critical bug) |
| V7.2 | Motor solo test Web UI, motor test page /motortest |
| V7.1 | Tăng Yaw P=1.0, I=0.30 chống xoay tròn |
| V6.6 | D on measurement, anti-windup, 250Hz stable |
| V5.0 | Double buffer ESP-NOW, I2C recovery, smooth motor |

---

## 👤 Tác giả

**tungdepzaivcl-123**  
GitHub: https://github.com/tungdepzaivcl-123/Drone_IOT_esp32

---

## 📄 License

MIT License — Tự do sử dụng, học tập, chia sẻ.
