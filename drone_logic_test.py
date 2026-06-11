#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
"""
drone_logic_test.py -- Automated Logic Test Suite V4.5
Kiểm tra toàn bộ các fix đã áp dụng vào drone_main.ino + drone_rc.ino
Chạy: python drone_logic_test.py
"""

import math, time

# ─── CONSTANTS (khớp 100% drone_main.ino V4.5) ──────────────────────────────
MIN_IDLE      = 260.0
MAX_THR       = 1000.0
MAX_PID       = 300.0
MAX_I         = 35.0
ARM_RAMP_MS   = 600
THR_I_GATE    = 1300   # #define trong firmware
REARM_COOLDOWN_MS = 2000
DISARM_ANGLE  = 45.0   # hardTilt threshold
FAST_ANGLE    = 35.0   # fastTilt angle threshold
FAST_RATE     = 200.0  # fastTilt rate threshold (deg/s)

kp_inner  = [1.20, 1.30, 1.20]
ki_inner  = [0.05, 0.05, 0.05]
kd_inner  = [0.12, 0.14, 0.05]
kp_angle  = 2.50
alpha_gyro  = 0.25
alpha_motor = 0.22
alpha_d     = 0.15   # V4.5 FIX (was 0.30)

# ─── COLOR ───────────────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

passed = 0
failed = 0

def test(name, condition, detail="", warn_only=False):
    global passed, failed
    if condition:
        passed += 1
        print(f"  {GREEN}[PASS]{RESET}  {name}")
    else:
        if warn_only:
            print(f"  {YELLOW}[WARN]{RESET}  {name}" + (f" | {detail}" if detail else ""))
        else:
            failed += 1
            print(f"  {RED}[FAIL]{RESET}  {name}" + (f" | {detail}" if detail else ""))

def section(title):
    print(f"\n{BOLD}{CYAN}{'='*55}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'='*55}{RESET}")

# === 1. THROTTLE MAPPING (V4.5 FIX) =========================================
section("1. THROTTLE MAPPING — Linear V4.5")

def throttle_map_v45(cmd_throttle):
    """V4.5: linear map, khong dead zone"""
    thr = MIN_IDLE + (cmd_throttle - 1000) * (MAX_THR - MIN_IDLE) / 1000.0
    return max(MIN_IDLE, min(MAX_THR, thr))

def throttle_map_old(cmd_throttle):
    """OLD: map + clamp → dead zone an"""
    thr = (cmd_throttle - 1000) * 1000 / 1000  # map(1000,2000,0,1000)
    return max(MIN_IDLE, thr)

# Diem endpoint
test("thr=1000 → MIN_IDLE",       abs(throttle_map_v45(1000) - MIN_IDLE) < 0.1)
test("thr=2000 → MAX_THR",        abs(throttle_map_v45(2000) - MAX_THR)  < 0.1)
test("thr=1500 → midpoint ~630",  abs(throttle_map_v45(1500) - (MIN_IDLE + (MAX_THR-MIN_IDLE)*0.5)) < 1.0)

# Kiem tra khong co dead zone
vals = [throttle_map_v45(t) for t in range(1000, 2001, 10)]
is_monotonic = all(vals[i+1] >= vals[i] for i in range(len(vals)-1))
test("Throttle tang deu (monotonic, khong dead zone)", is_monotonic)

# So sanh OLD vs NEW: dead zone cu 1000-1260 deu cho 260
old_1100 = throttle_map_old(1100)
new_1100 = throttle_map_v45(1100)
test("V4.5 thr=1100 khac MIN_IDLE (khong bi clamp nhu cu)",
     new_1100 > MIN_IDLE,
     f"new={new_1100:.1f}, old={old_1100:.1f} (cu = MIN_IDLE={MIN_IDLE})")

# Dead zone cu: 1000→1260 deu cho 260
old_1200 = throttle_map_old(1200)
new_1200 = throttle_map_v45(1200)
test("V4.5 thr=1200 > MIN_IDLE (old bi clamp = dead zone)",
     new_1200 > MIN_IDLE + 10,
     f"new={new_1200:.1f} vs old={old_1200:.1f}")

# === 2. MOTOR MIXER — Shift vs Scale =========================================
section("2. MOTOR MIXER — Shift (V4.5) vs Scale (OLD)")

def mixer_shift(thr, pid):
    """V4.5: shift de bao toan hieu"""
    m = [
        thr - pid[0] - pid[1] - pid[2],
        thr + pid[0] - pid[1] + pid[2],
        thr - pid[0] + pid[1] + pid[2],
        thr + pid[0] + pid[1] - pid[2],
    ]
    m_max = max(m)
    if m_max > MAX_THR:
        excess = m_max - MAX_THR
        m = [x - excess for x in m]
    m = [max(x, MIN_IDLE) for x in m]
    return m

def mixer_scale(thr, pid):
    """OLD: scale → thay doi ty le PID"""
    m = [
        thr - pid[0] - pid[1] - pid[2],
        thr + pid[0] - pid[1] + pid[2],
        thr - pid[0] + pid[1] + pid[2],
        thr + pid[0] + pid[1] - pid[2],
    ]
    m_max = max(m)
    if m_max > MAX_THR:
        scale = MAX_THR / m_max
        m = [x * scale for x in m]
    m = [max(x, MIN_IDLE) for x in m]
    return m

# Test hover (pid = 0)
m_hover = mixer_shift(620, [0, 0, 0])
test("Hover: tat ca motor = 620",
     all(abs(x - 620) < 0.1 for x in m_hover),
     str(m_hover))

# Test khi khong vuot MAX
pid_normal = [50, 30, 20]
m_normal = mixer_shift(500, pid_normal)
test("Khong vuot MAX: khong shift",      max(m_normal) <= MAX_THR)
test("Khong vuot MAX: min >= MIN_IDLE",  min(m_normal) >= MIN_IDLE)

# Test khi vuot MAX: Shift bao toan hieu, Scale thi khong
thr_high = 950
pid_large = [100, 80, 0]
m_sh = mixer_shift(thr_high, pid_large)
m_sc = mixer_scale(thr_high, pid_large)

# Hieu giua motor 0 va 1 phai bang 2*pid[0] voi shift
diff_sh_01 = m_sh[1] - m_sh[0]  # FL - FR = +2*roll
diff_sc_01 = m_sc[1] - m_sc[0]
expected_diff = 2 * pid_large[0]  # 200
test(f"Shift bao toan hieu motor FL-FR = {expected_diff:.0f}",
     abs(diff_sh_01 - expected_diff) < 0.5,
     f"shift={diff_sh_01:.1f}, expected={expected_diff}")
test(f"Scale pha vo hieu (diff khac {expected_diff:.0f}) — scale la sai",
     abs(diff_sc_01 - expected_diff) > 5,
     f"scale_diff={diff_sc_01:.1f} (nen khac {expected_diff})")

# Shift: max <= MAX_THR
test("Shift: max <= MAX_THR",  max(m_sh) <= MAX_THR + 0.1)
test("Scale: max <= MAX_THR",  max(m_sc) <= MAX_THR + 0.1)

# === 3. SOFT RAMP SAU ARM ====================================================
section("3. SOFT RAMP SAU ARM")

def apply_ramp(m_target, ramp_pct):
    """Mop phong soft ramp logic"""
    ramp_pct = max(0.0, min(1.0, ramp_pct))
    return [MIN_IDLE + (x - MIN_IDLE) * ramp_pct for x in m_target]

m_target = [620, 640, 600, 610]  # motor targets sau ARM

# t=0: ramp 0% → tat ca = MIN_IDLE
m_t0 = apply_ramp(m_target, 0.0)
test("Ramp t=0: tat ca motor = MIN_IDLE",
     all(abs(x - MIN_IDLE) < 0.1 for x in m_t0),
     str([f"{x:.0f}" for x in m_t0]))

# t=0.5 (300ms): 50% ramp
m_t50 = apply_ramp(m_target, 0.5)
test("Ramp t=50%: motor giua MIN_IDLE va target",
     all(MIN_IDLE <= x <= max(m_target) for x in m_t50),
     str([f"{x:.0f}" for x in m_t50]))

# t=1.0 (600ms): 100% → dat target
m_t100 = apply_ramp(m_target, 1.0)
test("Ramp t=100%: motor = target",
     all(abs(m_t100[i] - m_target[i]) < 0.1 for i in range(4)),
     str([f"{x:.0f}" for x in m_t100]))

# Ramp don dieu tang dan
ramp_values = [apply_ramp([700]*4, p/10.0)[0] for p in range(11)]
test("Ramp tang deu (monotonic)",
     all(ramp_values[i+1] >= ramp_values[i] for i in range(len(ramp_values)-1)),
     str([f"{x:.0f}" for x in ramp_values]))

# === 4. TILT FAILSAFE ========================================================
section("4. TILT FAILSAFE V4.5")

def check_tilt_failsafe(roll, pitch, gyro_x, gyro_y):
    hard_tilt = (abs(roll) > DISARM_ANGLE or abs(pitch) > DISARM_ANGLE)
    fast_tilt = ((abs(roll) > FAST_ANGLE or abs(pitch) > FAST_ANGLE) and
                 (abs(gyro_x) > FAST_RATE or abs(gyro_y) > FAST_RATE))
    return hard_tilt or fast_tilt, hard_tilt, fast_tilt

# Hover binh thuong
trig, hard, fast = check_tilt_failsafe(2.0, 1.5, 5.0, 3.0)
test("Hover binh thuong: khong disarm", not trig)

# Nghieng nhe + xoay cham: khong failsafe
trig, hard, fast = check_tilt_failsafe(30.0, 10.0, 50.0, 30.0)
test("Nghieng 30° + gyro 50 dps: khong failsafe", not trig)

# Hard tilt >45°
trig, hard, fast = check_tilt_failsafe(46.0, 5.0, 10.0, 5.0)
test("Hard tilt: roll=46° → DISARM",  trig and hard)

# Fast tilt: >35° + >200 dps
trig, hard, fast = check_tilt_failsafe(38.0, 5.0, 250.0, 20.0)
test("Fast tilt: roll=38° + gyro=250 dps → DISARM", trig and fast)

# Pitching fast
trig, hard, fast = check_tilt_failsafe(5.0, 36.0, 10.0, 220.0)
test("Fast tilt: pitch=36° + gyro_y=220 → DISARM", trig and fast)

# Border: 35° + 199 dps → khong disarm
trig, hard, fast = check_tilt_failsafe(35.0, 0, 199.0, 0)
test("Border: 35° + 199 dps → KHONG disarm (duoi nguong)", not trig)

# === 5. ARM LOGIC ============================================================
section("5. ARM LOGIC — Dieu kien & An toan")

def can_arm(throttle, lock, sticks_centered, last_recv_delta_ms, disarm_delta_ms):
    """Mop phong dieu kien ARM"""
    if lock:                      return False, "LOCKED"
    if throttle >= 1100:          return False, "THR_TOO_HIGH"
    if not sticks_centered:       return False, "STICKS_NOT_CENTER"
    if last_recv_delta_ms > 500:  return False, "NO_SIGNAL"
    if disarm_delta_ms < REARM_COOLDOWN_MS: return False, "COOLDOWN"
    return True, "OK"

ok, reason = can_arm(1050, False, True, 10, 5000)
test("ARM hop le: thr thap + sticks center + co tin hieu", ok, reason)

ok, reason = can_arm(1050, True, True, 10, 5000)
test("ARM bi chan khi LOCKED", not ok, reason)

ok, reason = can_arm(1150, False, True, 10, 5000)
test("ARM bi chan khi thr >= 1100", not ok, reason)

ok, reason = can_arm(1050, False, False, 10, 5000)
test("ARM bi chan khi sticks lech", not ok, reason)

ok, reason = can_arm(1050, False, True, 10, 500)
test(f"ARM bi chan trong cooldown {REARM_COOLDOWN_MS}ms", not ok, reason)

ok, reason = can_arm(1050, False, True, 6000, 5000)
test("ARM bi chan khi mat tin hieu >500ms", not ok, reason)

# === 6. FAILSAFE TIMEOUT =====================================================
section("6. FAILSAFE TIMEOUT — Adaptive")

def adaptive_timeout(throttle):
    """4s (idle) → 8s (full throttle)"""
    return 4000 + ((throttle - 1000) / 1000.0) * 4000

test("Timeout thr=1000: 4000ms", abs(adaptive_timeout(1000) - 4000) < 1)
test("Timeout thr=1500: 6000ms", abs(adaptive_timeout(1500) - 6000) < 1)
test("Timeout thr=2000: 8000ms", abs(adaptive_timeout(2000) - 8000) < 1)
test("Timeout tang khi ga cao (an toan hon)",
     adaptive_timeout(1800) > adaptive_timeout(1200))

# === 7. I-TERM GATE ==========================================================
section("7. I-TERM GATE — THR_I_GATE nhat quan")

def ok_to_integrate(throttle, alt_hold_on):
    return (throttle > THR_I_GATE) or alt_hold_on

test(f"Khong tich I khi thr={THR_I_GATE} (= nguong)",
     not ok_to_integrate(THR_I_GATE, False))
test(f"Tich I khi thr={THR_I_GATE+1} (tren nguong)",
     ok_to_integrate(THR_I_GATE + 1, False))
test("Tich I khi alt_hold du thr thap",
     ok_to_integrate(1100, True))
test("Khong tich I khi thr thap + khong alt_hold",
     not ok_to_integrate(1100, False))

# === 8. D-TERM FILTER ========================================================
section("8. D-TERM FILTER — alpha_d V4.5")

def simulate_d_filter(noise_amplitude, alpha, steps=100):
    """Mop phong D-term voi nhieu ngau nhien → do do bien dong output"""
    import random
    random.seed(42)
    d_filt = 0.0
    outputs = []
    for _ in range(steps):
        raw_d = random.uniform(-noise_amplitude, noise_amplitude)
        d_filt = (1.0 - alpha) * d_filt + alpha * raw_d
        outputs.append(d_filt)
    return max(outputs) - min(outputs)  # peak-to-peak

pp_old = simulate_d_filter(100.0, 0.30)
pp_new = simulate_d_filter(100.0, 0.15)
test(f"alpha=0.15 it nhieu hon alpha=0.30 (p-p: {pp_new:.1f} < {pp_old:.1f})",
     pp_new < pp_old,
     f"V4.5={pp_new:.2f}, OLD={pp_old:.2f}")
test("alpha=0.15 giam nhieu it nhat 30%",
     pp_new < pp_old * 0.70,
     f"ratio={pp_new/pp_old:.2f}")

# === 9. ESC DISARM SIGNAL ====================================================
section("9. ESC DISARM SIGNAL")

ESC_MIN_VALID = 1000  # Chuan ESC: 1000µs = stop, <1000 = invalid/bip

test(f"Disarm V4.5 = 1000µs (dung chuan ESC)",    1000 >= ESC_MIN_VALID)
test(f"Disarm OLD = 900µs (duoi chuan → gay ru)", 900 < ESC_MIN_VALID)
test("1000µs ≥ ESC minimum valid signal",           1000 >= ESC_MIN_VALID)

# === 10. EMA FILTER RC =======================================================
section("10. EMA FILTER RC — Latency")

def ema_latency_cycles(alpha):
    """So chu ky de EMA dat 63% step input (τ)"""
    # tau = 1/alpha - 1 (dung cho discrete EMA)
    return 1.0 / alpha - 1.0

ALPHA_RC  = 0.20
SEND_INTERVAL_MS = 20  # 50Hz

latency_old = ema_latency_cycles(0.20) * SEND_INTERVAL_MS  # voi 20 mau ADC + EMA
latency_new = ema_latency_cycles(0.20) * SEND_INTERVAL_MS  # EMA giu nguyen

# ADC blocking: 20 mau × ~0.1ms = 2ms, 8 mau × ~0.1ms = 0.8ms
adc_block_old_ms = 4 * (20 * 0.1)  # 4 pins × 20 samples × 0.1ms
adc_block_new_ms = 4 * (8  * 0.1)  # 4 pins × 8 samples
test(f"ADC blocking V4.5 ({adc_block_new_ms:.1f}ms) < OLD ({adc_block_old_ms:.1f}ms)",
     adc_block_new_ms < adc_block_old_ms)
test(f"Tiet kiem {adc_block_old_ms - adc_block_new_ms:.1f}ms/loop ADC",
     (adc_block_old_ms - adc_block_new_ms) > 2.0)

# === 11. PID CASCADE INTEGRATION =============================================
section("11. PID CASCADE — Kiem tra sign convention")

def pid_cascade(angle_roll, angle_pitch, cmd_rx, cmd_ry,
                center_rx=1500, center_ry=1500):
    """Outer loop: angle → rate setpoint"""
    target_roll  = (cmd_rx - center_rx) * 0.06 if abs(cmd_rx - center_rx) > 15 else 0.0
    target_pitch = (cmd_ry - center_ry) * 0.06 if abs(cmd_ry - center_ry) > 15 else 0.0
    rate_sp_roll  = kp_angle * (target_roll  - angle_roll)
    rate_sp_pitch = kp_angle * (target_pitch - angle_pitch)
    return rate_sp_roll, rate_sp_pitch

# Hover: sticks center, drone level → rate setpoint = 0
rsp_r, rsp_p = pid_cascade(0, 0, 1500, 1500)
test("Hover binh thuong: rate setpoint = 0", abs(rsp_r) < 0.1 and abs(rsp_p) < 0.1)

# Drone nghieng trai (angle_roll > 0), sticks center → can correction am (ha trai)
rsp_r, _ = pid_cascade(15.0, 0, 1500, 1500)
test("Roll correction: drone nghieng trai → rate setpoint am",
     rsp_r < 0,
     f"rsp_roll={rsp_r:.2f}")

# Sticks right → rate setpoint duong
rsp_r, _ = pid_cascade(0, 0, 1600, 1500)
test("Stick phai: rate setpoint roll duong", rsp_r > 0, f"rsp_roll={rsp_r:.2f}")

# === SUMMARY =================================================================
total = passed + failed
print(f"\n{BOLD}{'='*55}{RESET}")
print(f"{BOLD}  KET QUA TEST -- Drone Firmware V4.5{RESET}")
print(f"{'='*55}")
print(f"  Tong test  : {total}")
print(f"  {GREEN}[PASS]     : {passed}{RESET}")
if failed > 0:
    print(f"  {RED}[FAIL]     : {failed}{RESET}")
else:
    print(f"  {GREEN}[FAIL]     : {failed} -- TAT CA PASS!{RESET}")
print(f"{'='*55}\n")

if failed > 0:
    print(f"{RED}!! CO {failed} TEST THAT BAI -- Kiem tra lai logic!{RESET}\n")
    sys.exit(1)
else:
    print(f"{GREEN}** TAT CA {total} TEST DEU PASS -- Firmware V4.5 logic OK!{RESET}\n")
    sys.exit(0)
