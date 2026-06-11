"""
MÔ PHỎNG DRONE THỰC TẾ v2 — CÓ NHIỄU ĐẦY ĐỦ
================================================
Firmware: ESP32 + MPU6050 | Mass: 1.3kg | 250Hz

Các nguồn nhiễu được mô phỏng:
  [N1] Gyro white noise       (MPU6050: ~0.8 °/s rms)
  [N2] Gyro bias drift        (random walk ~0.05 °/s/s)
  [N3] Vibration motor        (harmonic noise theo RPM)
  [N4] DLPF 42Hz hardware     (1st-order low-pass)
  [N5] Wind gust              (lực ngẫu nhiên + gió liên tục)
  [N6] ESP-NOW packet loss    (1-5% drop + burst loss)
  [N7] ESC deadband & delay   (1 cycle delay, 2% deadband)
  [N8] Motor inertia          (1st-order spin-up ~30ms)

Motor layout từ motor test code:
  FR=M1(27), FL=M2(26), RR=M3(14), RL=M4(13)
  Roll+: M1,M3 tăng (cánh phải) → drone nghiêng trái
  Pitch+: M1,M2 tăng (cánh trước) → drone ngửa ra sau
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import warnings
warnings.filterwarnings('ignore')

rng = np.random.default_rng(42)

# ═══════════════════════════════════════════
# 1. VẬT LÝ DRONE (1.3kg)
# ═══════════════════════════════════════════
MASS    = 1.3
GRAVITY = 9.81
L_ARM   = 0.225   # m
Ixx     = 0.013   # kg·m²
Iyy     = 0.013
Izz     = 0.024

K_THRUST  = 1.275e-5   # N/(unit²) — phi tuyến như ESC thực
K_TORQUE  = 0.016      # m (reaction torque arm)
DRAG_LIN  = 0.25       # N·s/m
DRAG_ROT  = 0.003      # N·m·s/rad

# ═══════════════════════════════════════════
# 2. FIRMWARE PARAMS (copy nguyên)
# ═══════════════════════════════════════════
DT   = 0.004
FREQ = 250.0

kp = np.array([0.700993, 0.700993, 1.456010])
ki = np.array([1.265000, 1.265000, 1.401011])
kd = np.array([0.112501, 0.112501, 0.138702])

ALPHA_D   = 0.012
ALPHA_EMA = 0.2
MAX_PID   = 400.0
MAX_I     = 75.0
MIN_IDLE  = 40.0
MAX_THR   = 1000.0

# ═══════════════════════════════════════════
# 3. NOISE PARAMETERS
# ═══════════════════════════════════════════
# [N1] Gyro white noise — MPU6050 datasheet: ~0.05 °/s/√Hz @ 250Hz
GYRO_NOISE_STD  = 0.05 * np.sqrt(250)    # ≈ 0.79 °/s rms per axis

# [N2] Gyro bias drift — random walk
GYRO_DRIFT_RATE = 0.08   # °/s per second (drift tốc độ)

# [N3] Vibration — biên độ theo throttle (motor RPM)
VIB_AMPLITUDE   = 1.5    # °/s peak (khi full throttle)
VIB_FREQ_BASE   = 120.0  # Hz (harmonic cơ bản của motor)

# [N4] DLPF 42Hz — hệ số 1st-order IIR
# fc=42Hz, fs=250Hz → α = 2πfc/(2πfc + fs) ≈ 0.513
DLPF_ALPHA      = 2 * np.pi * 42 / (2 * np.pi * 42 + FREQ)

# [N5] Wind
WIND_STEADY     = np.array([0.8, 0.3, 0.0])   # N (gió liên tục)
WIND_GUST_STD   = 1.2   # N rms (gió giật ngẫu nhiên)
WIND_GUST_TAU   = 1.5   # s (time constant gió giật)

# [N6] ESP-NOW packet loss
PACKET_LOSS_RATE  = 0.02   # 2% random loss
BURST_LOSS_PROB   = 0.005  # 0.5% mỗi bước bắt đầu burst
BURST_LOSS_LEN    = 8      # burst mất tối đa 8 gói (~32ms)

# [N7] ESC deadband
ESC_DEADBAND      = 8.0   # ±8 units quanh điểm zero-change
ESC_DELAY_STEPS   = 1     # 1 bước delay (4ms)

# [N8] Motor inertia — 1st order với tau=30ms
MOTOR_TAU         = 0.030  # s
MOTOR_ALPHA       = 1 - np.exp(-DT / MOTOR_TAU)   # ≈ 0.124

# ═══════════════════════════════════════════
# 4. SCENARIO BAY
# ═══════════════════════════════════════════
class FlightScenario:
    PHASES = [
        (1.0,  1000, 1500, 1500, 1500),  # Pre-arm
        (2.5,  1450, 1500, 1500, 1500),  # Arm + ga nhẹ
        (5.0,  1560, 1500, 1500, 1500),  # Hover
        (6.5,  1560, 1650, 1500, 1500),  # Roll phải 52.5°/s
        (8.0,  1560, 1500, 1500, 1500),  # Level
        (9.5,  1560, 1500, 1650, 1500),  # Pitch tới
        (11.0, 1560, 1500, 1500, 1500),  # Level
        (12.5, 1560, 1500, 1500, 1350),  # Yaw trái
        (14.0, 1560, 1500, 1500, 1500),  # Dừng Yaw
        (15.5, 1300, 1500, 1500, 1500),  # Hạ cánh
        (16.5, 1000, 1500, 1500, 1500),  # Disarm
    ]
    def get(self, t):
        for t_end, *vals in self.PHASES:
            if t <= t_end: return vals
        return list(self.PHASES[-1][1:])

# ═══════════════════════════════════════════
# 5. TRẠNG THÁI KHỞI TẠO
# ═══════════════════════════════════════════
scenario = FlightScenario()

pos     = np.zeros(3)
vel     = np.zeros(3)
euler   = np.zeros(3)
omega   = np.zeros(3)      # rad/s (thực)

# PID state
integral_i  = np.zeros(3)
prev_err_i  = np.zeros(3)
deriv_i     = np.zeros(3)

# Noise state
gyro_bias       = np.zeros(3)        # [N2] Bias drift hiện tại
dlpf_state      = np.zeros(3)        # [N4] DLPF filter state
wind_gust       = np.zeros(3)        # [N5] Wind gust state
motor_actual    = np.zeros(4)        # [N8] Motor speed thực sau inertia
motor_prev_cmd  = np.zeros(4)        # [N7] ESC delay buffer
burst_counter   = 0                   # [N6] Burst loss counter
last_cmd        = [1000, 1500, 1500, 1500]  # [N6] Lệnh cuối (giữ khi mất gói)
smooth_gyro     = np.zeros(3)        # EMA state (như firmware)

isArmed = False

# ═══════════════════════════════════════════
# 6. SIM LOOP
# ═══════════════════════════════════════════
T_TOTAL = 16.5
N_STEPS = int(T_TOTAL / DT)

# Cấp phát lịch sử
H = {
    't':           np.zeros(N_STEPS),
    'pos':         np.zeros((N_STEPS, 3)),
    'euler':       np.zeros((N_STEPS, 3)),
    'gyro_true':   np.zeros((N_STEPS, 3)),    # tốc độ góc thực
    'gyro_raw':    np.zeros((N_STEPS, 3)),    # gyro có nhiễu thô
    'gyro_dlpf':   np.zeros((N_STEPS, 3)),    # sau DLPF hardware
    'gyro_ema':    np.zeros((N_STEPS, 3)),    # sau EMA software (vào PID)
    'pid_out':     np.zeros((N_STEPS, 3)),
    'motors_cmd':  np.zeros((N_STEPS, 4)),    # lệnh mixer
    'motors_act':  np.zeros((N_STEPS, 4)),    # thực tế (sau inertia)
    'wind':        np.zeros((N_STEPS, 3)),
    'armed':       np.zeros(N_STEPS, dtype=bool),
    'pkt_lost':    np.zeros(N_STEPS, dtype=bool),
    'throttle':    np.zeros(N_STEPS),
}

for step in range(N_STEPS):
    t = step * DT
    thr_pulse_ideal, rx_ideal, ry_ideal, lx_ideal = scenario.get(t)

    # ─── [N6] ESP-NOW Packet Loss ───
    pkt_lost = False
    if burst_counter > 0:
        pkt_lost = True; burst_counter -= 1
    elif rng.random() < PACKET_LOSS_RATE:
        pkt_lost = True
    elif rng.random() < BURST_LOSS_PROB:
        pkt_lost = True; burst_counter = rng.integers(2, BURST_LOSS_LEN)

    if not pkt_lost:
        last_cmd = [thr_pulse_ideal, rx_ideal, ry_ideal, lx_ideal]

    thr_pulse, rx, ry, lx = last_cmd
    thr_unit = float(np.clip(thr_pulse - 1000, 0, 1000))

    # ─── Arming logic (giống firmware) ───
    if not isArmed and thr_pulse <= 1000:
        pass
    if not isArmed and thr_pulse > 1000 and thr_pulse < 1060:
        isArmed = True
        integral_i[:] = 0; prev_err_i[:] = 0; deriv_i[:] = 0
    if thr_pulse <= 1000:
        isArmed = False

    # ─── [N2] Gyro bias drift ───
    gyro_bias += rng.normal(0, GYRO_DRIFT_RATE * np.sqrt(DT), 3)
    gyro_bias = np.clip(gyro_bias, -5, 5)   # giới hạn realistic

    # ─── [N3] Vibration scale theo throttle ───
    vib_scale = thr_unit / 1000.0
    vib = np.zeros(3)
    for h in [1, 2, 3]:  # 3 harmonics
        phase = rng.random(3) * 2 * np.pi
        vib += (VIB_AMPLITUDE * vib_scale / h) * np.sin(
            2 * np.pi * VIB_FREQ_BASE * h * t + phase)

    # ─── [N1] Gyro white noise ───
    white_noise = rng.normal(0, GYRO_NOISE_STD, 3)

    # ─── Gyro quan sát thực = omega_deg + bias + vibration + white_noise ───
    omega_deg_true = np.degrees(omega)
    gyro_raw = omega_deg_true + gyro_bias + vib + white_noise

    # ─── [N4] DLPF 42Hz hardware filter (IIR 1st-order) ───
    dlpf_state = DLPF_ALPHA * gyro_raw + (1 - DLPF_ALPHA) * dlpf_state

    # ─── EMA software filter (giống firmware: alpha=0.2) ───
    smooth_gyro = ALPHA_EMA * dlpf_state + (1 - ALPHA_EMA) * smooth_gyro

    # ─── Failsafe lật ───
    if abs(np.degrees(euler[0])) > 60 or abs(np.degrees(euler[1])) > 60:
        isArmed = False

    pid_out = np.zeros(3)
    motor_cmd = np.zeros(4)

    if isArmed:
        # ─── Rate targets ───
        target_rate = np.array([
            (rx   - 1500) * 0.35,
            (ry   - 1500) * 0.35,
            (lx   - 1500) * 0.60,
        ])

        # ─── PID inner loop ───
        for i in range(3):
            err = target_rate[i] - smooth_gyro[i]
            integral_i[i] = np.clip(integral_i[i] + err * DT, -MAX_I, MAX_I)
            raw_d = (err - prev_err_i[i]) * FREQ
            deriv_i[i] = (1 - ALPHA_D) * deriv_i[i] + ALPHA_D * raw_d
            pid_out[i] = np.clip(
                kp[i]*err + ki[i]*integral_i[i] + kd[i]*deriv_i[i],
                -MAX_PID, MAX_PID)
            prev_err_i[i] = err

        # ─── Mixer ───
        eff_thr = max(thr_unit, MIN_IDLE)
        motor_cmd[0] = eff_thr + pid_out[0] + pid_out[1] + pid_out[2]  # FR
        motor_cmd[1] = eff_thr - pid_out[0] + pid_out[1] - pid_out[2]  # FL
        motor_cmd[2] = eff_thr + pid_out[0] - pid_out[1] - pid_out[2]  # RR
        motor_cmd[3] = eff_thr - pid_out[0] - pid_out[1] + pid_out[2]  # RL
        motor_cmd = np.clip(motor_cmd, 0, MAX_THR)

        # ─── [N7] ESC deadband ───
        delta = motor_cmd - motor_prev_cmd
        delta[np.abs(delta) < ESC_DEADBAND] = 0
        motor_cmd = motor_prev_cmd + delta

    # ─── [N7] ESC delay (1 step) ───
    motor_delayed = motor_prev_cmd.copy()
    motor_prev_cmd = motor_cmd.copy()

    # ─── [N8] Motor inertia (1st-order) ───
    motor_actual = motor_actual + MOTOR_ALPHA * (motor_delayed - motor_actual)

    # ═══ VẬT LÝ 6-DOF ═══
    F_motors = K_THRUST * motor_actual**2   # Phi tuyến
    F_total  = np.sum(F_motors)

    cr, sr = np.cos(euler[0]), np.sin(euler[0])
    cp, sp = np.cos(euler[1]), np.sin(euler[1])
    cy, sy = np.cos(euler[2]), np.sin(euler[2])
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,             cp*cr            ],
    ])
    thrust_world = R @ np.array([0, 0, F_total])

    # ─── [N5] Wind ───
    wind_gust += (-wind_gust * DT / WIND_GUST_TAU +
                  rng.normal(0, WIND_GUST_STD * np.sqrt(DT / WIND_GUST_TAU), 3))
    wind_total = WIND_STEADY + wind_gust
    wind_total[2] = 0.0   # gió nằm ngang

    accel_world = (np.array([0, 0, -GRAVITY])
                   + thrust_world / MASS
                   + wind_total / MASS
                   - DRAG_LIN * vel / MASS)

    F0, F1, F2, F3 = F_motors
    tau_roll  = (F0 + F2 - F1 - F3) * L_ARM
    tau_pitch = (F0 + F1 - F2 - F3) * L_ARM
    tau_yaw   = (F0 + F3 - F1 - F2) * K_TORQUE

    alpha_rot = np.array([
        (tau_roll  - DRAG_ROT * omega[0]) / Ixx,
        (tau_pitch - DRAG_ROT * omega[1]) / Iyy,
        (tau_yaw   - DRAG_ROT * omega[2]) / Izz,
    ])

    omega += alpha_rot * DT
    euler += omega * DT
    vel   += accel_world * DT
    pos   += vel * DT
    pos[2] = max(pos[2], 0.0)
    if pos[2] == 0 and vel[2] < 0: vel[2] = 0

    # ─── Lưu ───
    H['t'][step]          = t
    H['pos'][step]        = pos.copy()
    H['euler'][step]      = np.degrees(euler)
    H['gyro_true'][step]  = omega_deg_true.copy()
    H['gyro_raw'][step]   = gyro_raw.copy()
    H['gyro_dlpf'][step]  = dlpf_state.copy()
    H['gyro_ema'][step]   = smooth_gyro.copy()
    H['pid_out'][step]    = pid_out.copy()
    H['motors_cmd'][step] = motor_cmd.copy()
    H['motors_act'][step] = motor_actual.copy()
    H['wind'][step]       = wind_total.copy()
    H['armed'][step]      = isArmed
    H['pkt_lost'][step]   = pkt_lost
    H['throttle'][step]   = thr_pulse

t_ax = H['t']
n_lost = H['pkt_lost'].sum()
print(f"✅ {N_STEPS} steps | Max alt: {H['pos'][:,2].max():.2f}m | "
      f"Packets lost: {n_lost}/{N_STEPS} ({100*n_lost/N_STEPS:.1f}%)")

# ═══════════════════════════════════════════════════════
# 7. VẼ ĐỒ THỊ
# ═══════════════════════════════════════════════════════
DARK_BG  = '#0a0a18'
PANEL_BG = '#10101e'
armed_mask = H['armed']

fig = plt.figure(figsize=(20, 13), facecolor=DARK_BG)
fig.suptitle(
    '🚁  Mô Phỏng Drone Thực Tế v2  —  Firmware ESP32 + 8 Nguồn Nhiễu  |  Mass=1.3kg | 250Hz',
    color='white', fontsize=13, y=0.99)

gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.55, wspace=0.35,
                       left=0.06, right=0.98, top=0.95, bottom=0.05)

CLR = {
    'roll':  '#ff6b6b', 'pitch': '#ffd93d', 'yaw': '#6bcb77',
    'alt':   '#4d96ff', 'raw':   '#ff4444', 'dlpf':'#ffaa00',
    'ema':   '#00ffcc', 'true':  '#ffffff',
    'm': ['#ff6b6b','#ffd93d','#6bcb77','#4d96ff']
}

def style(ax, title, ylabel='', ylim=None, xlabel=True):
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors='#666688', labelsize=7)
    ax.set_title(title, color='#ccccee', fontsize=8, pad=3)
    if ylabel: ax.set_ylabel(ylabel, color='#666688', fontsize=7)
    if xlabel: ax.set_xlabel('t (s)', color='#666688', fontsize=7)
    ax.grid(True, color='#18182e', linewidth=0.5, linestyle=':')
    for sp in ax.spines.values(): sp.set_edgecolor('#1e1e3a')
    if ylim: ax.set_ylim(ylim)
    # Vùng armed
    in_a = False; sa = 0
    for i in range(len(t_ax)):
        if armed_mask[i] and not in_a:   in_a=True;  sa=t_ax[i]
        elif not armed_mask[i] and in_a: ax.axvspan(sa, t_ax[i], alpha=0.07, color='lime'); in_a=False
    if in_a: ax.axvspan(sa, t_ax[-1], alpha=0.07, color='lime')
    # Packet loss markers
    lost_t = t_ax[H['pkt_lost']]
    if len(lost_t) > 0:
        ax.vlines(lost_t, *ax.get_ylim(), colors='#ff000033', lw=0.4)


# ── Row 0: Gyro pipeline (cho Roll) ──────────────────
ax_g = [fig.add_subplot(gs[0, i]) for i in range(3)]

for ax, axis_i, label, title in zip(
    ax_g, [0,1,2], ['Roll','Pitch','Yaw'],
    ['🌀 Gyro Roll — Raw vs DLPF vs EMA (PID input)',
     '🌀 Gyro Pitch — Raw vs DLPF vs EMA',
     '🌀 Gyro Yaw — Raw vs DLPF vs EMA']):
    style(ax, title, '°/s')
    ax.plot(t_ax, H['gyro_raw'][:,axis_i],  color=CLR['raw'],  lw=0.5, alpha=0.5, label='Raw (+noise)')
    ax.plot(t_ax, H['gyro_dlpf'][:,axis_i], color=CLR['dlpf'], lw=0.8, alpha=0.8, label='DLPF 42Hz')
    ax.plot(t_ax, H['gyro_ema'][:,axis_i],  color=CLR['ema'],  lw=1.2, label='EMA (vào PID)')
    ax.plot(t_ax, H['gyro_true'][:,axis_i], color=CLR['true'], lw=0.8, ls='--', alpha=0.6, label='Thực')
    ax.legend(fontsize=6, facecolor=PANEL_BG, labelcolor='white', loc='upper right', ncol=2)

# ── Row 1: Attitude + PID ─────────────────────────────
ax_roll = fig.add_subplot(gs[1, 0])
style(ax_roll, '📐 Góc Roll / Pitch', '°', (-40, 40))
ax_roll.plot(t_ax, H['euler'][:,0], color=CLR['roll'],  lw=1.2, label='Roll')
ax_roll.plot(t_ax, H['euler'][:,1], color=CLR['pitch'], lw=1.2, label='Pitch')
ax_roll.axhline(0, color='white', lw=0.4, ls='--', alpha=0.3)
ax_roll.legend(fontsize=7, facecolor=PANEL_BG, labelcolor='white')

ax_yaw = fig.add_subplot(gs[1, 1])
style(ax_yaw, '🧭 Góc Yaw', '°')
ax_yaw.plot(t_ax, H['euler'][:,2], color=CLR['yaw'], lw=1.2)

ax_pid = fig.add_subplot(gs[1, 2])
style(ax_pid, '⚙️ PID Output', 'units')
ax_pid.plot(t_ax, H['pid_out'][:,0], color=CLR['roll'],  lw=0.9, label='Roll')
ax_pid.plot(t_ax, H['pid_out'][:,1], color=CLR['pitch'], lw=0.9, label='Pitch')
ax_pid.plot(t_ax, H['pid_out'][:,2], color=CLR['yaw'],   lw=0.9, label='Yaw')
ax_pid.legend(fontsize=7, facecolor=PANEL_BG, labelcolor='white')

# ── Row 2: Motors + Altitude + Wind ──────────────────
ax_mot = fig.add_subplot(gs[2, 0])
style(ax_mot, '🔧 Motor Speed (Cmd vs Thực — Motor Inertia)', 'units')
labels = ['FR(M1)','FL(M2)','RR(M3)','RL(M4)']
for i in range(4):
    ax_mot.plot(t_ax, H['motors_cmd'][:,i], color=CLR['m'][i], lw=0.6, alpha=0.5, ls='--')
    ax_mot.plot(t_ax, H['motors_act'][:,i], color=CLR['m'][i], lw=1.1, label=labels[i])
ax_mot.legend(fontsize=7, facecolor=PANEL_BG, labelcolor='white', ncol=2)
ax_mot.text(0.99, 0.02, '-- Lệnh mixer   — Thực (τ=30ms)',
    transform=ax_mot.transAxes, color='#aaaaaa', fontsize=6,
    ha='right', va='bottom')

ax_alt = fig.add_subplot(gs[2, 1])
style(ax_alt, '📊 Độ Cao', 'm')
ax_alt.plot(t_ax, H['pos'][:,2], color=CLR['alt'], lw=1.5)
ax_alt.fill_between(t_ax, H['pos'][:,2], alpha=0.15, color=CLR['alt'])

ax_wind = fig.add_subplot(gs[2, 2])
style(ax_wind, '💨 Lực Gió (Wind Force)', 'N')
ax_wind.plot(t_ax, H['wind'][:,0], color='#88aaff', lw=0.9, label='Wx')
ax_wind.plot(t_ax, H['wind'][:,1], color='#aaffcc', lw=0.9, label='Wy')
ax_wind.axhline(WIND_STEADY[0], color='#88aaff', lw=0.5, ls=':', alpha=0.5)
ax_wind.axhline(WIND_STEADY[1], color='#aaffcc', lw=0.5, ls=':', alpha=0.5)
ax_wind.legend(fontsize=7, facecolor=PANEL_BG, labelcolor='white')

# ── Row 3: 3D trajectory + noise summary ─────────────
ax3d = fig.add_subplot(gs[3, :2], projection='3d')
ax3d.set_facecolor(PANEL_BG)
ax3d.set_title('🗺️  Quỹ Đạo Bay 3D (bị ảnh hưởng bởi nhiễu & gió)', color='#ccccee', fontsize=9)
ax3d.set_xlabel('X', color='#666688', fontsize=7); ax3d.set_ylabel('Y', color='#666688', fontsize=7)
ax3d.set_zlabel('Z', color='#666688', fontsize=7); ax3d.tick_params(colors='#666688', labelsize=6)
ax3d.xaxis.pane.fill = False; ax3d.yaxis.pane.fill = False; ax3d.zaxis.pane.fill = False
ax3d.xaxis.pane.set_edgecolor('#1a1a2e'); ax3d.yaxis.pane.set_edgecolor('#1a1a2e'); ax3d.zaxis.pane.set_edgecolor('#1a1a2e')
x3,y3,z3 = H['pos'][:,0], H['pos'][:,1], H['pos'][:,2]
for i in range(0, N_STEPS-2, 4):
    c = plt.cm.plasma(i/N_STEPS)
    end = min(i+5, N_STEPS)
    ax3d.plot(x3[i:end], y3[i:end], z3[i:end], color=c, lw=1.2, alpha=0.85)
ax3d.scatter([x3[0]],[y3[0]],[z3[0]], color='lime', s=40)
ax3d.scatter([x3[-1]],[y3[-1]],[z3[-1]], color='red', s=40)

# ── Noise legend box ─────────────────────────────────
ax_info = fig.add_subplot(gs[3, 2])
ax_info.set_facecolor(PANEL_BG)
ax_info.set_axis_off()
noise_summary = [
    ("[N1] Gyro white noise",     f"{GYRO_NOISE_STD:.2f} °/s rms"),
    ("[N2] Gyro bias drift",      f"{GYRO_DRIFT_RATE} °/s²"),
    ("[N3] Vibration (motor)",    f"±{VIB_AMPLITUDE} °/s @ {VIB_FREQ_BASE}Hz"),
    ("[N4] DLPF 42Hz hardware",   f"α={DLPF_ALPHA:.3f} (IIR)"),
    ("[N5] Wind steady",           f"{WIND_STEADY[:2]} N"),
    ("     Wind gust",            f"σ={WIND_GUST_STD} N, τ={WIND_GUST_TAU}s"),
    ("[N6] Packet loss",           f"{PACKET_LOSS_RATE*100:.0f}% + burst"),
    ("     Packets lost",         f"{n_lost}/{N_STEPS} ({100*n_lost/N_STEPS:.1f}%)"),
    ("[N7] ESC deadband",         f"±{ESC_DEADBAND} units, delay 1 step"),
    ("[N8] Motor inertia",        f"τ={MOTOR_TAU*1000:.0f}ms"),
]
for row, (k,v) in enumerate(noise_summary):
    clr = '#ffcc44' if k.startswith('[') else '#888899'
    ax_info.text(0.02, 1 - row*0.095, k, color=clr,    fontsize=7.5, fontfamily='monospace', transform=ax_info.transAxes, va='top')
    ax_info.text(0.55, 1 - row*0.095, v, color='#aaccff', fontsize=7.5, fontfamily='monospace', transform=ax_info.transAxes, va='top')

ax_info.set_title('📋 Noise Sources Summary', color='#ccccee', fontsize=9)

plt.savefig('drone_noise_sim.png', dpi=110, bbox_inches='tight', facecolor=DARK_BG)
print("📸 Saved: drone_noise_sim.png")
plt.show()
