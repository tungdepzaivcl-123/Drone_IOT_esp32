#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== CẤU HÌNH WIFI (TỰ PHÁT) =====
const char* ap_ssid     = "ESP32_Joystick_RC"; // Tên WiFi phát ra
const char* ap_password = "12345678";          // Mật khẩu (ít nhất 8 ký tự)

WebServer server(80);

// ===== CẤU HÌNH OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
#define I2C_SDA       5    
#define I2C_SCL       6    

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== CHÂN JOYSTICK =====
#define JOY_L_X  0
#define JOY_L_Y  1
#define JOY_R_X  2
#define JOY_R_Y  3

// ===== CHÂN NÚT =====
#define BTN_MODE     7
#define BTN_UNLOCK   8
#define BTN_ALTHOLD  10
#define BTN_X        11

// ===== HTML WEB =====
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Controller</title>
<style>
  body { font-family: monospace; background: #0f0f1a; color: #e0e0e0; padding: 20px; text-align: center; }
  h1 { color: #00d4ff; }
  .card { background: #1a1a2e; border: 1px solid #2a2a4a; border-radius: 10px; padding: 16px; max-width: 400px; margin: 10px auto; }
  .val { font-size: 1.3em; color: #00d4ff; margin: 6px 0; }
  .bar-wrap { background: #0a0a18; border-radius: 6px; height: 12px; margin: 4px 0; }
  .bar { height: 100%; border-radius: 6px; background: linear-gradient(90deg,#00d4ff,#7b2fff); }
  .btns { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 400px; margin: 10px auto; }
  .btn { padding: 14px; border-radius: 8px; border: 2px solid #2a2a4a; background: #1a1a2e; font-size: 0.9em; }
  .btn.on { background: #00d4ff22; border-color: #00d4ff; color: #00d4ff; }
</style>
<script>
  setInterval(() => {
    fetch('/data').then(r => r.json()).then(d => {
      document.getElementById('lx').textContent = d.lx;
      document.getElementById('ly').textContent = d.ly;
      document.getElementById('rx').textContent = d.rx;
      document.getElementById('ry').textContent = d.ry;
      document.getElementById('bar-lx').style.width = (d.lx/4095*100)+'%';
      document.getElementById('bar-ly').style.width = (d.ly/4095*100)+'%';
      document.getElementById('bar-rx').style.width = (d.rx/4095*100)+'%';
      document.getElementById('bar-ry').style.width = (d.ry/4095*100)+'%';
      document.getElementById('bm').className = 'btn'+(d.m?' on':'');
      document.getElementById('bu').className = 'btn'+(d.u?' on':'');
      document.getElementById('ba').className = 'btn'+(d.a?' on':'');
      document.getElementById('bx').className = 'btn'+(d.x?' on':'');
    });
  }, 200);
</script>
</head>
<body>
<h1>🎮 ESP32 ACCESS POINT</h1>
<div class="card">
  <b>JOYSTICK TRÁI</b>
  <div class="val">X: <span id="lx">-</span></div>
  <div class="bar-wrap"><div class="bar" id="bar-lx" style="width:50%"></div></div>
  <div class="val">Y: <span id="ly">-</span></div>
  <div class="bar-wrap"><div class="bar" id="bar-ly" style="width:50%"></div></div>
</div>
<div class="card">
  <b>JOYSTICK PHẢI</b>
  <div class="val">X: <span id="rx">-</span></div>
  <div class="bar-wrap"><div class="bar" id="bar-rx" style="width:50%"></div></div>
  <div class="val">Y: <span id="ry">-</span></div>
  <div class="bar-wrap"><div class="bar" id="bar-ry" style="width:50%"></div></div>
</div>
<div class="btns">
  <div class="btn" id="bm">MODE</div>
  <div class="btn" id="bu">UNLOCK</div>
  <div class="btn" id="ba">ALT HOLD</div>
  <div class="btn" id="bx">X</div>
</div>
</body>
</html>
)rawliteral";

int lx, ly, rx, ry;
bool bm, bu, ba, bx;

void readSensors() {
  lx = analogRead(JOY_L_X);
  ly = analogRead(JOY_L_Y);
  rx = analogRead(JOY_R_X);
  ry = analogRead(JOY_R_Y);
  bm = !digitalRead(BTN_MODE);
  bu = !digitalRead(BTN_UNLOCK);
  ba = !digitalRead(BTN_ALTHOLD);
  bx = !digitalRead(BTN_X);
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 0);
  display.println("AP MODE ACTIVE");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  display.setCursor(0, 12);
  display.printf("L X:%4d Y:%4d", lx, ly);
  display.drawRect(0, 22, 128, 4, SSD1306_WHITE);
  display.fillRect(0, 22, map(lx, 0, 4095, 0, 128), 4, SSD1306_WHITE);

  display.setCursor(0, 29);
  display.printf("R X:%4d Y:%4d", rx, ry);
  display.drawRect(0, 39, 128, 4, SSD1306_WHITE);
  display.fillRect(0, 39, map(rx, 0, 4095, 0, 128), 4, SSD1306_WHITE);

  display.setCursor(0, 46);
  display.printf("M:%d U:%d A:%d X:%d", bm, bu, ba, bx);

  display.setCursor(0, 56);
  display.printf("IP: %s", WiFi.softAPIP().toString().c_str());
  display.display();
}

void handleRoot() { server.send(200, "text/html", HTML); }

void handleData() {
  String json = "{\"lx\":" + String(lx) + ",\"ly\":" + String(ly) +
                ",\"rx\":" + String(rx) + ",\"ry\":" + String(ry) +
                ",\"m\":"  + String(bm) + ",\"u\":"  + String(bu) +
                ",\"a\":"  + String(ba) + ",\"x\":"  + String(bx) + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_MODE,    INPUT_PULLUP);
  pinMode(BTN_UNLOCK,  INPUT_PULLUP);
  pinMode(BTN_ALTHOLD, INPUT_PULLUP);
  pinMode(BTN_X,       INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display.clearDisplay();
    display.setCursor(10, 20);
    display.println("Khoi tao WiFi AP...");
    display.display();
  }

  // Cấu hình phát WiFi
  WiFi.softAP(ap_ssid, ap_password);
  
  Serial.println("\n--- WiFi AP Status ---");
  Serial.print("SSID: "); Serial.println(ap_ssid);
  Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("WIFI PHAT OK!");
  display.setCursor(0, 25);
  display.println(ap_ssid);
  display.setCursor(0, 40);
  display.println(WiFi.softAPIP().toString());
  display.display();
  delay(2000);
}

void loop() {
  server.handleClient();
  static unsigned long lastOLED = 0;
  if (millis() - lastOLED > 100) { // Tăng tốc độ cập nhật lên 100ms
    lastOLED = millis();
    readSensors();
    updateOLED();
  }
}