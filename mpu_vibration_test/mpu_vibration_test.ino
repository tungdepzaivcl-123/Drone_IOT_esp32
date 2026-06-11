// ═══════════════════════════════════════════════════════════════
//  MPU6050 VIBRATION AUTO-TEST — V3.0
//  Tu dong chay qua cac muc throttle va bao cao nhieu
//
//  Hardware: SDA=21 SCL=22 | Motor: 27,26,14,13
//  Baud    : 115200
//  ⚠️  THAO CANH QUAT TRUOC KHI CAM PIN!
//
//  Quy trinh TU DONG:
//    Boot → Calibrate MPU (3s) → ESC arm (2s) →
//    Tu dong tang ga: 0% → 15% → 25% → 35% → 45%
//    Moi muc thu thap 6 giay → In bao cao → Tat motor
//    Cuoi cung in khuyen nghi alpha_gyro cho drone_main.ino
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <MPU6050_light.h>

// ── Phần cứng ─────────────────────────────────────────────────
#define SDA_PIN   21
#define SCL_PIN   22
const int motorPins[4] = {27, 26, 14, 13};  // FR FL RR RL

// ── ESC / PWM ─────────────────────────────────────────────────
#define PWM_FREQ   250
#define PWM_BITS   12
#define PWM_PER    4000   // us (1/250Hz)
#define P_OFF      1000
#define P_MAX      1450   // MAX an toan = 45% (khong du luc nang drone)

// ── Chuoi test tu dong ────────────────────────────────────────
const int  THR[]      = {0, 15, 25, 35, 45};  // % throttle
const int  N_THR      = 5;
const int  COLLECT_S  = 6;    // giay thu thap du lieu moi muc
const int  RAMP_MS    = 1500; // ms doi on dinh truoc khi thu thap

// ── Bo loc EMA test song song ──────────────────────────────────
// Thu 5 muc alpha cung luc de tim alpha tot nhat
const float  AL[]   = {0.90f, 0.70f, 0.50f, 0.30f, 0.15f};
const char*  ALN[]  = {"a0.90","a0.70","a0.50","a0.30","a0.15"};
const int    N_AL   = 5;

// ── State machine ─────────────────────────────────────────────
enum State { ST_CALIB, ST_ESC, ST_COUNTDOWN, ST_RAMP, ST_COLLECT, ST_REPORT, ST_DONE };
State         g_state    = ST_CALIB;
int           g_lvl      = 0;         // index vao THR[]
unsigned long g_stateMs  = 0;
unsigned long g_loopUs   = 0;

// ── MPU ───────────────────────────────────────────────────────
MPU6050 mpu(Wire);

// ── EMA filter state (per alpha) ──────────────────────────────
float fPitch[N_AL], fRoll[N_AL], fGX[N_AL];

// ── Welford online stddev (khong can luu toan bo mang) ────────
struct Stat {
  uint32_t n;
  double   mean, M2;
  float    vmin, vmax;
  void reset() { n=0; mean=0; M2=0; vmin=1e9; vmax=-1e9; }
  void add(float x) {
    n++;
    vmin = min(vmin, x); vmax = max(vmax, x);
    double d = x - mean; mean += d/n; M2 += d*(x-mean);
  }
  float std() { return (n>1) ? sqrtf(M2/(n-1)) : 0; }
  float rng() { return vmax - vmin; }
};

// Stats: [alpha_idx][axis: 0=pitch 1=roll 2=gyrX]
Stat  fStat[N_AL][3];
Stat  rStat[3];   // raw stats

// ── Luu ket qua moi muc throttle ─────────────────────────────
struct LvlResult {
  int   thr;
  float rawStdPitch, rawStdRoll, rawStdGX;
  float filtStd[N_AL];   // std pitch sau EMA moi alpha
};
LvlResult res[N_THR];
int       resCount = 0;

// ── Helpers ───────────────────────────────────────────────────
uint32_t duty(int us) { return (uint32_t)((us*4095UL)/PWM_PER); }

void setMotors(int pulse) {
  pulse = constrain(pulse, P_OFF, P_MAX);
  for (int i=0; i<4; i++) ledcWrite(motorPins[i], duty(pulse));
}

int thrPulse(int pct) {
  if (pct<=0) return P_OFF;
  return P_OFF + (int)((float)constrain(pct,0,45)*(P_MAX-P_OFF)/45.0f);
}

void applyDLPF(byte lvl) {
  // CONFIG reg: DLPF_CFG
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(lvl); Wire.endTransmission(); delay(5);
  // Gyro FS = ±250 deg/s
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); delay(5);
  // Accel FS = ±2g
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); delay(5);
  // Verify
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1); byte got = Wire.read();
  const int bw[] = {256,188,98,42,21,10,5};
  Serial.printf("[DLPF] 0x%02X → cutoff=%dHz %s\n", got, bw[constrain((int)got,0,6)], got==lvl?"OK":"FAIL!");
}

void resetStats() {
  for (int a=0;a<N_AL;a++) for (int x=0;x<3;x++) fStat[a][x].reset();
  for (int x=0;x<3;x++) rStat[x].reset();
}

void initFilters(float pitch, float roll, float gx) {
  for (int a=0;a<N_AL;a++) { fPitch[a]=pitch; fRoll[a]=roll; fGX[a]=gx; }
}

// ── In bang ket qua 1 muc ─────────────────────────────────────
void printLvlReport(int idx) {
  LvlResult& r = res[idx];
  Serial.println();
  Serial.printf("[RPT] =========== THR = %d%% (pulse %d) ===========\n", r.thr, thrPulse(r.thr));
  Serial.printf("[RPT] RAW noise  : Pitch=%.3f deg  Roll=%.3f deg  GyrX=%.2f deg/s\n",
    r.rawStdPitch, r.rawStdRoll, r.rawStdGX);
  Serial.println("[RPT] EMA filter (Pitch std deviation):");
  Serial.println("[RPT]   Alpha  | Std(deg) | Giam noise | Danh gia");
  for (int a=0;a<N_AL;a++) {
    float red = (r.rawStdPitch > 0) ? (1.0f - r.filtStd[a]/r.rawStdPitch)*100.0f : 0;
    const char* rat =
      r.filtStd[a] < 0.05f ? "XUAT SAC" :
      r.filtStd[a] < 0.15f ? "Rat tot" :
      r.filtStd[a] < 0.40f ? "Tot" :
      r.filtStd[a] < 1.0f  ? "Trung binh" :
                              "Kem";
    Serial.printf("[RPT]   %-6s | %8.4f | %5.1f%%     | %s\n",
      ALN[a], r.filtStd[a], red, rat);
  }
}

// ── In bao cao tong hop va khuyen nghi ───────────────────────
void printFinalReport() {
  Serial.println("\n[FINAL] ╔══════════════════════════════════════════════╗");
  Serial.println("[FINAL] ║      BAO CAO TONG HOP - RUNG DONG MPU       ║");
  Serial.println("[FINAL] ╚══════════════════════════════════════════════╝");

  // Bang so lieu theo throttle
  Serial.print("[FINAL]         ");
  for (int l=0;l<resCount;l++) Serial.printf(" THR%2d%%", res[l].thr);
  Serial.println();
  Serial.print("[FINAL] RAW-std ");
  for (int l=0;l<resCount;l++) Serial.printf(" %5.3f ", res[l].rawStdPitch);
  Serial.println("(deg)");
  for (int a=0;a<N_AL;a++) {
    Serial.printf("[FINAL] %-7s  ", ALN[a]);
    for (int l=0;l<resCount;l++) Serial.printf(" %5.3f ", res[l].filtStd[a]);
    Serial.println();
  }

  // Tim alpha tot nhat tai muc throttle cao nhat
  int topLvl = resCount-1;
  while (topLvl>0 && res[topLvl].thr==0) topLvl--;
  float bestStd=1e9; int bestA=2;
  for (int a=0;a<N_AL;a++) if (res[topLvl].filtStd[a]<bestStd) { bestStd=res[topLvl].filtStd[a]; bestA=a; }

  float maxRaw=0;
  for (int l=1;l<resCount;l++) maxRaw=max(maxRaw,res[l].rawStdPitch);

  Serial.println("[FINAL]");
  Serial.printf("[FINAL] Nhieu RAW max: %.3f deg std\n", maxRaw);

  if (maxRaw < 0.3f) {
    Serial.println("[FINAL] => PHAN CUNG CHONG RUNG XUAT SAC!");
    Serial.println("[FINAL]    MPU6050 cua ban rat on dinh khi motor chay.");
  } else if (maxRaw < 1.0f) {
    Serial.println("[FINAL] => Rung dong nhe. Phan cung OK.");
    Serial.println("[FINAL]    Nen: lot cao su mong duoi ESP32/MPU.");
  } else if (maxRaw < 3.0f) {
    Serial.println("[FINAL] => Rung dong trung binh.");
    Serial.println("[FINAL]    Can: lot cao su chong rung cho MPU (foam tape 2 mat day).");
    Serial.println("[FINAL]    Can: can bang canh quat (prop balancing).");
  } else {
    Serial.println("[FINAL] => RUNG DONG MANH - NGUY HIEM!");
    Serial.println("[FINAL]    Bat buoc lam truoc khi bay:");
    Serial.println("[FINAL]    1. Dat MPU/ESP32 tren lot cao su chuyen dung");
    Serial.println("[FINAL]    2. Can bang canh quat bang may can bang");
    Serial.println("[FINAL]    3. Siet chat oc vit motor, khung");
    Serial.println("[FINAL]    4. Kiem tra motor co bi lech truc khong");
  }

  Serial.println("[FINAL]");
  Serial.println("[FINAL] ─────── KHUYEN NGHI CHO drone_main.ino ──────");
  Serial.printf("[FINAL]   float alpha_gyro  = %.2ff;   // EMA gyro\n", AL[bestA]);
  Serial.printf("[FINAL]   float alpha_d     = %.2ff;   // EMA D-term\n",
    max(0.10f, AL[bestA]-0.10f));
  Serial.println("[FINAL]   // Trong setupMPUFilter(): giu DLPF=0x04 (21Hz)");
  Serial.println("[FINAL] ──────────────────────────────────────────────");
  Serial.printf("[FINAL]   => Alpha khuyen dung: %s (std=%.4f deg)\n", ALN[bestA], bestStd);
  Serial.println("[FINAL]");
  Serial.println("[FINAL] DONE. Reset ESP32 de chay lai. Upload drone_main voi gia tri tren.");
}

// ── Plotter header (in 1 lan khi bat dau collect) ────────────
void printPlotterHeader(int thr) {
  // Dong nay bi ignore boi Serial Plotter, chi hien thi trong Serial Monitor
  Serial.printf("[PLT-HDR] THR=%d%% | RawP FP_a90 FP_a70 FP_a50 FP_a30 FP_a15 RawR FR_a50\n", thr);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("[BOOT] ============================================");
  Serial.println("[BOOT]  MPU6050 Vibration AUTO-TEST v3.0");
  Serial.println("[BOOT]  THAO CANH QUAT CHUA? Kiem tra lan cuoi!");
  Serial.println("[BOOT] ============================================");
  delay(500);

  // Motor OFF ngay lap tuc
  for (int i=0;i<4;i++) {
    ledcAttach(motorPins[i], PWM_FREQ, PWM_BITS);
    ledcWrite(motorPins[i], duty(P_OFF));
  }
  Serial.println("[MOTOR] Tat ca motor OFF.");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(3);

  // MPU
  byte s = mpu.begin();
  if (s != 0) {
    Serial.printf("[ERR] MPU6050 LOI! code=%d. Kiem tra SDA=%d SCL=%d\n",s,SDA_PIN,SCL_PIN);
    while(1) { delay(1000); Serial.println("[ERR] HALT - MPU MISSING"); }
  }
  Serial.println("[MPU] MPU6050 tim thay.");

  // Calibrate
  Serial.println("[MPU] === CALIBRATE MPU ===");
  Serial.println("[MPU] Dat drone PHANG, DUNG RUNG! Bat dau sau 2 giay...");
  delay(2000);
  Serial.println("[MPU] Dang calibrate (khoang 3 giay)...");
  mpu.calcOffsets(true, true);
  Serial.println("[MPU] Calibrate XONG.");

  // DLPF (PHAI GOI SAU calcOffsets vi library reset no!)
  Serial.println("[MPU] Thiet lap DLPF phan cung (0x04 = 21Hz)...");
  applyDLPF(0x04);

  // Warm-up EMA
  mpu.update();
  float ip = mpu.getAngleX(), ir = mpu.getAngleY(), ig = mpu.getGyroX();
  initFilters(ip, ir, ig);
  for (int i=0;i<80;i++) {
    mpu.update();
    float p=mpu.getAngleX(), r=mpu.getAngleY(), gx=mpu.getGyroX();
    for (int a=0;a<N_AL;a++) {
      fPitch[a] = AL[a]*p + (1-AL[a])*fPitch[a];
      fRoll [a] = AL[a]*r + (1-AL[a])*fRoll [a];
      fGX   [a] = AL[a]*gx+ (1-AL[a])*fGX   [a];
    }
    delay(4);
  }

  // ESC arm
  Serial.println("[ESC] Dang arm ESC (2 giay)...");
  g_state   = ST_ESC;
  g_stateMs = millis();

  g_loopUs = micros();
}

// ── Loop 250Hz ────────────────────────────────────────────────
void loop() {
  // Timing
  unsigned long now = micros();
  if (now - g_loopUs < 4000) return;
  g_loopUs = now;

  // Doc MPU
  mpu.update();
  float rP  = mpu.getAngleX();
  float rR  = mpu.getAngleY();
  float rGX = mpu.getGyroX();

  // Cap nhat tat ca EMA
  for (int a=0;a<N_AL;a++) {
    fPitch[a] = AL[a]*rP  + (1.0f-AL[a])*fPitch[a];
    fRoll [a] = AL[a]*rR  + (1.0f-AL[a])*fRoll [a];
    fGX   [a] = AL[a]*rGX + (1.0f-AL[a])*fGX   [a];
  }

  unsigned long ms = millis();

  // ── State machine ────────────────────────────────────────
  switch (g_state) {

    // ── ESC arm: giu 1000us trong 2s ──────────────────────
    case ST_ESC:
      if (ms - g_stateMs >= 2000) {
        Serial.println("[ESC] ESC san sang!");
        Serial.println();
        Serial.println("[AUTO] ==========================================");
        Serial.println("[AUTO]  BAT DAU CHUOI TEST TU DONG");
        Serial.printf ("[AUTO]  %d muc throttle x %d giay = ~%d giay tong\n",
          N_THR, COLLECT_S, N_THR*(COLLECT_S+2)+5);
        Serial.println("[AUTO]  Serial Plotter: Pitch Raw + 5 muc loc EMA");
        Serial.println("[AUTO] ==========================================");
        g_lvl    = 0;
        g_state  = ST_COUNTDOWN;
        g_stateMs= ms;
      }
      break;

    // ── Countdown truoc moi muc ────────────────────────────
    case ST_COUNTDOWN: {
      // In dem nguoc 3 2 1 moi giay
      static int lastCnt = -1;
      int remaining = 3 - (int)((ms - g_stateMs)/1000);
      if (remaining != lastCnt && remaining > 0) {
        lastCnt = remaining;
        Serial.printf("[AUTO] Chuan bi: THR=%d%% bat sau %d giay...\n", THR[g_lvl], remaining);
      }
      if (ms - g_stateMs >= 3000) {
        lastCnt = -1;
        // Bat motor
        int pulse = thrPulse(THR[g_lvl]);
        setMotors(pulse);
        Serial.printf("[AUTO] >>> BAT THR=%d%% (pulse=%d) - Doi on dinh %.1fs...\n",
          THR[g_lvl], pulse, RAMP_MS/1000.0f);
        g_state  = ST_RAMP;
        g_stateMs= ms;
      }
      break;
    }

    // ── Ramp: cho motor on dinh truoc khi thu thap ─────────
    case ST_RAMP:
      if (ms - g_stateMs >= (unsigned long)RAMP_MS) {
        resetStats();
        initFilters(rP, rR, rGX);   // Reset EMA tu gia tri hien tai
        printPlotterHeader(THR[g_lvl]);
        Serial.printf("[AUTO] >>> Thu thap %d giay...\n", COLLECT_S);
        g_state  = ST_COLLECT;
        g_stateMs= ms;
      }
      break;

    // ── Collect: thu thap du lieu ──────────────────────────
    case ST_COLLECT:
      // Ghi stats
      rStat[0].add(rP);
      rStat[1].add(rR);
      rStat[2].add(rGX);
      for (int a=0;a<N_AL;a++) {
        fStat[a][0].add(fPitch[a]);
        fStat[a][1].add(fRoll [a]);
        fStat[a][2].add(fGX   [a]);
      }

      // Plotter output (moi buoc loop = 4ms = 250 dong/s — qua nhieu)
      // Chi in moi 20ms (50Hz) de Serial Plotter de xem
      {
        static unsigned long lastPlotMs = 0;
        if (ms - lastPlotMs >= 20) {
          lastPlotMs = ms;
          // Format: RawP FP(a0.9) FP(a0.7) FP(a0.5) FP(a0.3) FP(a0.15) RawR FR(a0.5) Thr
          Serial.print("RawP:"); Serial.print(rP,2);
          for (int a=0;a<N_AL;a++) {
            Serial.print("\t"); Serial.print(ALN[a]); Serial.print(":"); Serial.print(fPitch[a],2);
          }
          Serial.print("\tRawR:"); Serial.print(rR,2);
          Serial.print("\tFR_a50:"); Serial.print(fRoll[2],2);
          Serial.print("\tThr:"); Serial.print(THR[g_lvl]);
          Serial.println();
        }
      }

      // Hoan thanh muc nay?
      if (ms - g_stateMs >= (unsigned long)(COLLECT_S*1000)) {
        // Luu ket qua
        LvlResult& r = res[resCount];
        r.thr = THR[g_lvl];
        r.rawStdPitch = rStat[0].std();
        r.rawStdRoll  = rStat[1].std();
        r.rawStdGX    = rStat[2].std();
        for (int a=0;a<N_AL;a++) r.filtStd[a] = fStat[a][0].std();
        resCount++;

        g_state  = ST_REPORT;
        g_stateMs= ms;
      }
      break;

    // ── Report muc hien tai ────────────────────────────────
    case ST_REPORT:
      // Tat motor truoc khi in
      setMotors(P_OFF);
      printLvlReport(resCount-1);
      delay(500);   // Cho in xong

      g_lvl++;
      if (g_lvl >= N_THR) {
        // Xong tat ca muc
        g_state  = ST_DONE;
        g_stateMs= ms;
      } else {
        g_state  = ST_COUNTDOWN;
        g_stateMs= ms;
      }
      break;

    // ── Done ──────────────────────────────────────────────
    case ST_DONE:
      if (ms - g_stateMs < 200) {   // Chi in 1 lan
        setMotors(P_OFF);
        printFinalReport();
      }
      // Loop idle, motor tat
      break;
  }
}
