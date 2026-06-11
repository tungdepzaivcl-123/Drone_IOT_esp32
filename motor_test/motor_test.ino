// ═══════════════════════════════════════════════════════════════
//  MOTOR TEST STANDALONE — V1.0
//  Sketch riêng để test từng motor qua Web UI
//  KHÔNG có PID, KHÔNG có bay — chỉ test phần cứng!
//
//  Board : ESP32 (dùng đúng board giống drone_main)
//  WiFi  : SSID="MotorTest" | Pass="motor1234"
//  Truy cập: http://192.168.4.1
//
//  Motor pins (GIỐNG drone_main):
//    M1 - FR (Trước Phải)  : GPIO 27  (CCW)
//    M2 - FL (Trước Trái)  : GPIO 26  (CW)
//    M3 - RR (Sau Phải)    : GPIO 14  (CW)
//    M4 - RL (Sau Trái)    : GPIO 13  (CCW)
//
//  ⚠️  THAO CANH QUAT TRUOC KHI TEST!
// ═══════════════════════════════════════════════════════════════
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi AP ──────────────────────────────────────────────────
#define AP_SSID   "MotorTest"
#define AP_PASS   "motor1234"

// ── Motor Pins ───────────────────────────────────────────────
#define PIN_M1_FR  27   // Front-Right  (CCW)
#define PIN_M2_FL  26   // Front-Left   (CW)
#define PIN_M3_RR  14   // Rear-Right   (CW)
#define PIN_M4_RL  13   // Rear-Left    (CCW)

const int motorPins[4] = { PIN_M1_FR, PIN_M2_FL, PIN_M3_RR, PIN_M4_RL };
const char* motorNames[4] = {
  "M1 - FR (Truoc Phai) CCW",
  "M2 - FL (Truoc Trai) CW",
  "M3 - RR (Sau Phai)   CW",
  "M4 - RL (Sau Trai)   CCW"
};

// ── LEDC (PWM) Config ────────────────────────────────────────
// ESC dùng PWM 50Hz (chu kỳ 20000µs), nhưng ta dùng 250Hz (4000µs)
// vì đây là ESC digital (đồng bộ với drone_main)
#define PWM_FREQ_HZ   250
#define PWM_BITS      12        // 12-bit = 0..4095
#define PWM_PERIOD_US 4000      // 1/250Hz = 4000µs

// Pulse limits (an toàn tuyệt đối — không đủ lực bay)
#define PULSE_OFF     1000      // Motor tắt (ESC armed idle)
#define PULSE_MIN     1070      // Motor vừa bắt đầu quay
#define PULSE_MAX     1250      // Tối đa cho test (< 30% power, không đủ lực nâng)
#define PULSE_DEFAULT 1100      // Mặc định khi bắt đầu test

// ── State ────────────────────────────────────────────────────
int  activeMotor  = -1;         // -1 = tắt tất cả, 0-3 = motor đang chạy
int  testPulse    = PULSE_DEFAULT;
unsigned long motorStartMs = 0;
#define AUTO_STOP_MS 15000      // Tự dừng sau 15 giây

WebServer server(80);

// ── Helpers ──────────────────────────────────────────────────
uint32_t pulseToDuty(int pulse_us) {
  return (uint32_t)((pulse_us * 4095UL) / PWM_PERIOD_US);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    ledcWrite(motorPins[i], pulseToDuty(PULSE_OFF));
  }
  activeMotor = -1;
  Serial.println("[MOTOR] All stopped (1000us)");
}

void runMotor(int idx, int pulse) {
  if (idx < 0 || idx > 3) return;
  pulse = constrain(pulse, PULSE_MIN, PULSE_MAX);

  // Dừng tất cả trước
  for (int i = 0; i < 4; i++) {
    ledcWrite(motorPins[i], pulseToDuty(PULSE_OFF));
  }
  delay(50);

  // Chạy motor được chọn
  ledcWrite(motorPins[idx], pulseToDuty(pulse));
  activeMotor   = idx;
  testPulse     = pulse;
  motorStartMs  = millis();

  Serial.printf("[MOTOR] Running %s at %d us\n", motorNames[idx], pulse);
}

// ── Web Handlers ─────────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", R"HTML(<!DOCTYPE html>
<html lang="vi"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Motor Test - Drone</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#060a10;color:#c8d8e8;font-family:'Segoe UI',sans-serif;padding:16px;max-width:440px;margin:0 auto}
h1{text-align:center;padding:18px 0 4px;font-size:1.5rem;font-weight:800;background:linear-gradient(90deg,#00d2ff,#a855f7);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;font-size:.75rem;color:#405060;margin-bottom:16px}
.warn{background:#200800;border:1px solid #ff4400;border-radius:10px;padding:12px 14px;font-size:.8rem;color:#ff7744;margin-bottom:16px;line-height:1.6;text-align:center;font-weight:700}
.status-bar{background:#091520;border:1px solid #1a3050;border-radius:10px;padding:10px 14px;margin-bottom:16px;display:flex;align-items:center;gap:10px;font-size:.8rem}
.dot{width:11px;height:11px;border-radius:50%;flex-shrink:0;transition:all .3s}
.dot-off{background:#1a1a1a}
.dot-on{background:#00ff88;box-shadow:0 0 10px #00ff8888}
.dot-warn{background:#ff8800;box-shadow:0 0 10px #ff880088}
.card{background:#0d1825;border:1px solid #1a3050;border-radius:12px;padding:18px;margin-bottom:16px}
.card h2{font-size:.95rem;font-weight:700;color:#80b0d0;margin-bottom:16px;display:flex;align-items:center;gap:8px}
/* Drone layout grid */
.drone-wrap{position:relative;width:260px;height:260px;margin:0 auto 16px}
.drone-body{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:80px;height:80px;background:#0d1825;border:2px solid #1a3050;border-radius:16px;display:flex;flex-direction:column;align-items:center;justify-content:center;font-size:.62rem;color:#405060;text-align:center;line-height:1.4}
.drone-arrow{position:absolute;top:6px;left:50%;transform:translateX(-50%);color:#00d2ff;font-size:1.2rem}
.mbtn{position:absolute;width:100px;height:90px;border-radius:14px;border:2px solid #1a4060;background:#081525;color:#44aaff;font-weight:700;font-size:.85rem;cursor:pointer;transition:all .2s;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:4px;line-height:1.3}
.mbtn:active{transform:scale(.93)}
.mbtn.active{border-color:#00d2ff;background:#001c35;box-shadow:0 0 20px #00d2ff55;color:#00eeff}
.mbtn .spin{font-size:.65rem;font-weight:400;color:#608090;margin-top:2px}
.mbtn .dir-badge{font-size:.6rem;padding:2px 6px;border-radius:4px;margin-top:3px}
.ccw{background:#0a2a1a;color:#00cc66;border:1px solid #00aa44}
.cw{background:#1a1a2a;color:#8888ff;border:1px solid #6666ee}
/* positions */
#mb0{top:8px;right:8px}
#mb1{top:8px;left:8px}
#mb2{bottom:8px;right:8px}
#mb3{bottom:8px;left:8px}
/* Pulse slider */
.pulse-section{display:flex;align-items:center;gap:12px}
.pulse-lbl{font-size:.75rem;color:#608080;min-width:45px}
.pulse-num{font-size:1.3rem;font-weight:800;color:#00d2ff;min-width:65px;text-align:right}
input[type=range]{flex:1;accent-color:#00d2ff;cursor:pointer}
/* Running status */
#run-info{margin-top:14px;padding:10px 14px;background:#0a1200;border:1px solid #225500;border-radius:8px;font-size:.8rem;color:#88dd44;text-align:center;min-height:36px;line-height:1.5;display:none}
#run-info.active{display:block}
/* Stop btn */
.btn-stop{width:100%;padding:15px;margin-top:6px;background:linear-gradient(135deg,#6b0000,#bb0000);border:none;border-radius:12px;color:#fff;font-size:1.05rem;font-weight:700;cursor:pointer;letter-spacing:.5px;transition:opacity .15s}
.btn-stop:hover{opacity:.85}
.btn-stop:active{transform:scale(.97)}
/* Timer bar */
.timer-wrap{height:4px;background:#0a1000;border-radius:2px;margin-top:10px;overflow:hidden;display:none}
.timer-wrap.active{display:block}
.timer-bar{height:100%;background:linear-gradient(90deg,#00d2ff,#a855f7);width:100%;transition:width .5s linear;border-radius:2px}
/* Pos labels */
.pos-lbl{position:absolute;font-size:.6rem;color:#506070;font-weight:600}
#lbl-fl{top:102px;left:0;text-align:center}
#lbl-fr{top:102px;right:0;text-align:center}
#lbl-rl{bottom:102px;left:0;text-align:center}
#lbl-rr{bottom:102px;right:0;text-align:center}
.log-box{background:#040810;border:1px solid #0d1a26;border-radius:8px;padding:8px 10px;font-size:.68rem;color:#3a5070;height:100px;overflow-y:auto;font-family:'Courier New',monospace;margin-top:12px}
a.back{display:block;text-align:center;margin:16px 0 8px;color:#304060;font-size:.75rem;text-decoration:none}
a.back:hover{color:#00d2ff}
</style></head><body>
<h1>🧪 Motor Test</h1>
<p class="sub">Nhan dien vi tri tung dong co — Thao canh quat truoc!</p>

<div class="warn">⚠️ THAO CANH QUAT TRUOC KHI CHAY MOTOR!<br>Pulse toi da: 1250µs — Khong du luc bay.</div>

<div class="status-bar">
  <div class="dot dot-on" id="wdot"></div>
  <span id="wtxt">Ket noi WiFi: MotorTest</span>
  &nbsp;&nbsp;
  <div class="dot dot-off" id="mdot"></div>
  <span id="mtxt">Motor: Tat</span>
</div>

<div class="card">
  <h2>📍 Chon Motor</h2>
  <div class="drone-wrap">
    <!-- Position labels -->
    <span class="pos-lbl" id="lbl-fl">TRUOC<br>TRAI</span>
    <span class="pos-lbl" id="lbl-fr">TRUOC<br>PHAI</span>
    <span class="pos-lbl" id="lbl-rl">SAU<br>TRAI</span>
    <span class="pos-lbl" id="lbl-rr">SAU<br>PHAI</span>
    <!-- Motor buttons -->
    <button class="mbtn" id="mb1" onclick="testMotor(1)">
      M2 - FL
      <span class="dir-badge cw">CW ↻</span>
      <span class="spin">Truoc Trai</span>
    </button>
    <button class="mbtn" id="mb0" onclick="testMotor(0)">
      M1 - FR
      <span class="dir-badge ccw">CCW ↺</span>
      <span class="spin">Truoc Phai</span>
    </button>
    <button class="mbtn" id="mb3" onclick="testMotor(3)">
      M4 - RL
      <span class="dir-badge ccw">CCW ↺</span>
      <span class="spin">Sau Trai</span>
    </button>
    <button class="mbtn" id="mb2" onclick="testMotor(2)">
      M3 - RR
      <span class="dir-badge cw">CW ↻</span>
      <span class="spin">Sau Phai</span>
    </button>
    <!-- Body -->
    <div class="drone-body">
      <div class="drone-arrow">▲</div>
      DRONE<br>BODY<br><small style="color:#2a4050">(dau)</small>
    </div>
  </div>

  <div id="run-info">Chua chay motor nao. Bam nut de bat dau.</div>
  <div class="timer-wrap" id="timer-wrap">
    <div class="timer-bar" id="timer-bar"></div>
  </div>
</div>

<div class="card">
  <h2>⚡ Dieu chinh Pulse</h2>
  <div class="pulse-section">
    <span class="pulse-lbl">Pulse</span>
    <input type="range" id="slider" min="1070" max="1250" value="1100" step="5"
           oninput="onSlider(this.value)">
    <span class="pulse-num" id="pnum">1100 µs</span>
  </div>
  <button class="btn-stop" onclick="stopAll()">⏹ DUNG TAT CA MOTOR</button>
</div>

<div class="log-box" id="log">Log...<br></div>
<a class="back" href="#">⚙ Drone Motor Test v1.0</a>

<script>
const AUTO_MS = 15000;
let curMotor = -1, curPulse = 1100;
let timerStart = 0, timerIv = null;
const names = ['M1-FR (Truoc Phai)','M2-FL (Truoc Trai)','M3-RR (Sau Phai)','M4-RL (Sau Trai)'];
const btns  = [0,1,2,3].map(i => document.getElementById('mb'+i));

function addLog(msg) {
  const el = document.getElementById('log');
  const t  = new Date().toLocaleTimeString();
  el.innerHTML = t + ' ' + msg + '<br>' + el.innerHTML;
}

function setMotorDot(on) {
  const d = document.getElementById('mdot');
  const t = document.getElementById('mtxt');
  d.className = on ? 'dot dot-on' : 'dot dot-off';
  t.textContent = on ? 'Motor: DANG CHAY' : 'Motor: Tat';
}

function highlightBtn(idx) {
  btns.forEach((b, i) => {
    b.className = i === idx ? 'mbtn active' : 'mbtn';
  });
}

function startTimer() {
  timerStart = Date.now();
  document.getElementById('timer-wrap').className = 'timer-wrap active';
  document.getElementById('run-info').className = 'active';
  if (timerIv) clearInterval(timerIv);
  timerIv = setInterval(() => {
    const pct = Math.max(0, 100 - (Date.now() - timerStart) * 100 / AUTO_MS);
    document.getElementById('timer-bar').style.width = pct + '%';
    if (pct <= 0) {
      clearInterval(timerIv); timerIv = null;
      handleStopped();
    }
  }, 200);
}

function handleStopped() {
  curMotor = -1;
  highlightBtn(-1);
  setMotorDot(false);
  document.getElementById('timer-wrap').className = 'timer-wrap';
  document.getElementById('run-info').textContent = 'Da dung (het gio). Bam lai de test.';
  addLog('AUTO-STOP sau 15s');
}

function testMotor(idx) {
  curPulse = parseInt(document.getElementById('slider').value);
  fetch('/run?m=' + idx + '&p=' + curPulse)
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        curMotor = idx;
        highlightBtn(idx);
        setMotorDot(true);
        document.getElementById('run-info').textContent =
          '▶ Dang chay: ' + names[idx] + ' — ' + curPulse + ' µs (15s)';
        startTimer();
        addLog('BAT: ' + names[idx] + ' | ' + curPulse + ' µs');
      } else {
        addLog('LOI: ' + (d.msg || '?'));
      }
    })
    .catch(e => addLog('Loi ket noi: ' + e));
}

function onSlider(v) {
  curPulse = parseInt(v);
  document.getElementById('pnum').textContent = v + ' µs';
  if (curMotor >= 0) testMotor(curMotor); // cap nhat pulse ngay
}

function stopAll() {
  fetch('/stop')
    .then(r => r.json())
    .then(() => {
      if (timerIv) { clearInterval(timerIv); timerIv = null; }
      handleStopped();
      document.getElementById('run-info').textContent = 'Da dung tat ca motor.';
      addLog('STOP tat ca motor');
    })
    .catch(e => addLog('Loi: ' + e));
}
</script>
</body></html>)HTML");
}

void handleRun() {
  if (!server.hasArg("m")) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"missing m\"}");
    return;
  }
  int idx   = server.arg("m").toInt();
  int pulse = server.hasArg("p") ? server.arg("p").toInt() : PULSE_DEFAULT;

  if (idx < 0 || idx > 3) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"invalid motor\"}");
    return;
  }

  runMotor(idx, pulse);

  char buf[80];
  snprintf(buf, sizeof(buf),
    "{\"ok\":true,\"m\":%d,\"pulse\":%d,\"name\":\"%s\"}",
    idx + 1, testPulse, motorNames[idx]);
  server.send(200, "application/json", buf);
}

void handleStop() {
  stopAllMotors();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStatus() {
  char buf[120];
  snprintf(buf, sizeof(buf),
    "{\"active\":%d,\"pulse\":%d,\"uptime\":%lu}",
    activeMotor, testPulse, millis() / 1000);
  server.send(200, "application/json", buf);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Motor Test Standalone v1.0");
  Serial.println("  ⚠️  THAO CANH QUAT TRUOC KHI TEST!");

  // LEDC init cho 4 motor
  for (int i = 0; i < 4; i++) {
    ledcAttach(motorPins[i], PWM_FREQ_HZ, PWM_BITS);
    ledcWrite(motorPins[i], pulseToDuty(PULSE_OFF));
    Serial.printf("  [LEDC] Pin %d -> 1000us (off)\n", motorPins[i]);
  }

  // Chờ ESC arm (1.5 giây)
  Serial.println("[ESC] Waiting for ESC to arm (1.5s)...");
  delay(1500);
  Serial.println("[ESC] ESC armed OK");

  // WiFi AP
  WiFi.softAP(AP_SSID, AP_PASS, 1);
  delay(300);
  Serial.printf("[AP] SSID: %s | IP: %s\n", AP_SSID,
    WiFi.softAPIP().toString().c_str());
  Serial.printf("[AP] Mo browser: http://%s\n",
    WiFi.softAPIP().toString().c_str());

  // Web routes
  server.on("/",       handleRoot);
  server.on("/run",    handleRun);
  server.on("/stop",   handleStop);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("[WEB] Server started");
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  // Auto-stop sau AUTO_STOP_MS
  if (activeMotor >= 0 && (millis() - motorStartMs > AUTO_STOP_MS)) {
    Serial.printf("[AUTO-STOP] Motor %d stopped after %ds\n",
      activeMotor + 1, AUTO_STOP_MS / 1000);
    stopAllMotors();
  }
}
