"""
MO PHONG DRONE BAY 3D - v2
===========================
- Drone 1.3kg khop voi firmware thuc te
- PID Position Hold + Altitude
- Animation 3D real-time
- Fix: khong dung emoji (tranh canh bao font)
- Giao dien dep hon, hien thi motor speed
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.colors import to_rgba
import warnings
warnings.filterwarnings('ignore')

# ─────────────────────────────────────────────
# 1. THONG SO VAT LY (khop drone thuc: 1.3kg)
# ─────────────────────────────────────────────
DT       = 0.05      # buoc thoi gian (s)
MASS     = 1.3       # kg - drone thuc te
GRAVITY  = 9.81      # m/s^2
MAX_THRUST = 35.0    # luc day toi da (N)
MAX_TILT   = 0.30    # goc nghieng toi da (rad ~17 do) - giam de tranh overshoot
DRAG       = 0.70    # he so can gio - TANG MANH de phanh drone nang hon
MAX_VEL_XY = 2.5    # van toc ngang toi da (m/s)
MAX_VEL_Z  = 1.5    # van toc doc toi da (m/s)

# PID ngang (X, Y) - position loop
KP_XY = 0.40;  KI_XY = 0.010;  KD_XY = 0.55

# PID do cao (Z)
KP_Z  = 1.6;   KI_Z  = 0.03;   KD_Z  = 1.0

# ─────────────────────────────────────────────
# 2. WAYPOINTS (x, y, z)
# ─────────────────────────────────────────────
WAYPOINTS = np.array([
    [ 0,  0,  0],    # WP0: cat canh
    [ 0,  0,  3],    # WP1: len cao 3m
    [ 5,  0,  3],    # WP2: bay thang
    [ 5,  5,  4],    # WP3: re va len
    [ 0,  5,  4],    # WP4: re trai
    [ 0,  0,  4],    # WP5: quay ve
    [ 2.5,2.5,5.5],  # WP6: dinh cao
    [ 0,  0,  1],    # WP7: ha xuong
    [ 0,  0,  0],    # WP8: ha canh
], dtype=float)

REACH_RADIUS = 0.25

# ─────────────────────────────────────────────
# 3. PID CLASS
# ─────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, i_limit=None):
        self.kp=kp; self.ki=ki; self.kd=kd
        self.i_limit=i_limit
        self.integral=0.0; self.prev_error=0.0

    def update(self, error, dt):
        self.integral += error * dt
        if self.i_limit:
            self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*derivative

# ─────────────────────────────────────────────
# 4. SIM LOOP
# ─────────────────────────────────────────────
pid_x = PID(KP_XY, KI_XY, KD_XY, i_limit=5)
pid_y = PID(KP_XY, KI_XY, KD_XY, i_limit=5)
pid_z = PID(KP_Z,  KI_Z,  KD_Z,  i_limit=8)

pos     = WAYPOINTS[0].copy().astype(float)
vel     = np.zeros(3)
wp_idx  = 1

history_pos   = [pos.copy()]
history_wp    = [wp_idx]
history_roll  = [0.0]
history_pitch = [0.0]
history_motor = [np.ones(4) * 1000.0]  # ESC pulse 4 motor

HOVER_THROTTLE = 1560   # pulse tuong duong hover 1.3kg

MAX_STEPS = 4000

for step in range(MAX_STEPS):
    target = WAYPOINTS[wp_idx]
    err    = target - pos

    if np.linalg.norm(err) < REACH_RADIUS:
        if wp_idx < len(WAYPOINTS) - 1:
            wp_idx += 1
        else:
            for _ in range(30):
                history_pos.append(pos.copy())
                history_wp.append(wp_idx)
                history_roll.append(0.0)
                history_pitch.append(0.0)
                history_motor.append(np.ones(4) * HOVER_THROTTLE)
            break

    ax_cmd = np.clip(pid_x.update(err[0], DT), -MAX_TILT*GRAVITY, MAX_TILT*GRAVITY)
    ay_cmd = np.clip(pid_y.update(err[1], DT), -MAX_TILT*GRAVITY, MAX_TILT*GRAVITY)
    az_cmd = pid_z.update(err[2], DT)

    thrust = np.clip((az_cmd + GRAVITY) * MASS, 0, MAX_THRUST)
    accel  = np.array([ax_cmd, ay_cmd, thrust/MASS - GRAVITY])
    accel -= DRAG * vel

    vel += accel * DT
    # Gioi han van toc tranh overshoot
    vel[0] = np.clip(vel[0], -MAX_VEL_XY, MAX_VEL_XY)
    vel[1] = np.clip(vel[1], -MAX_VEL_XY, MAX_VEL_XY)
    vel[2] = np.clip(vel[2], -MAX_VEL_Z,  MAX_VEL_Z)
    pos  = pos + vel * DT
    pos[2] = max(pos[2], 0.0)

    pitch = np.arctan2(ax_cmd, GRAVITY)
    roll  = np.arctan2(-ay_cmd, GRAVITY)

    # Tinh motor (px) tu thrust + roll/pitch
    thr_unit = np.clip((thrust / MAX_THRUST) * 1000, 40, 1000)
    r_pid = np.degrees(roll)  * 1.2
    p_pid = np.degrees(pitch) * 1.2
    motors = np.array([
        1000 + np.clip(thr_unit + r_pid + p_pid, 0, 1000),  # FR
        1000 + np.clip(thr_unit - r_pid + p_pid, 0, 1000),  # FL
        1000 + np.clip(thr_unit + r_pid - p_pid, 0, 1000),  # RR
        1000 + np.clip(thr_unit - r_pid - p_pid, 0, 1000),  # RL
    ])

    history_pos.append(pos.copy())
    history_wp.append(wp_idx)
    history_roll.append(roll)
    history_pitch.append(pitch)
    history_motor.append(motors)

history_pos   = np.array(history_pos)
history_roll  = np.array(history_roll)
history_pitch = np.array(history_pitch)
history_motor = np.array(history_motor)
N = len(history_pos)

# ─────────────────────────────────────────────
# 5. DRONE ARMS
# ─────────────────────────────────────────────
ARM_LEN = 0.30

def drone_arms(cx, cy, cz, roll, pitch, yaw=0):
    arms_body = np.array([
        [ ARM_LEN, 0, 0], [-ARM_LEN, 0, 0],
        [0,  ARM_LEN, 0], [0, -ARM_LEN, 0],
    ])
    cr,sr = np.cos(roll), np.sin(roll)
    cp,sp = np.cos(pitch),np.sin(pitch)
    cy2,sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy2,-sy,0],[sy,cy2,0],[0,0,1]])
    R  = Rz @ Ry @ Rx
    return (R @ arms_body.T).T + np.array([cx,cy,cz])

# ─────────────────────────────────────────────
# 6. FIGURE: 3D (trai) + subplots (phai)
# ─────────────────────────────────────────────
DARK  = '#080812'
PANEL = '#0e0e1e'
GRID  = '#181830'

fig = plt.figure(figsize=(16, 9), facecolor=DARK)
fig.suptitle('DRONE FLIGHT SIMULATION  |  1.3 kg  |  9 Waypoints',
             color='#ccddff', fontsize=13, y=0.99, fontweight='bold')

gs = gridspec.GridSpec(3, 2, figure=fig,
                       left=0.03, right=0.98, top=0.95, bottom=0.05,
                       hspace=0.45, wspace=0.08,
                       width_ratios=[2.2, 1])

# Subplot 3D chinh
ax3d = fig.add_subplot(gs[:, 0], projection='3d')
ax3d.set_facecolor(PANEL)
ax3d.set_xlim(-1, 6); ax3d.set_ylim(-1, 6); ax3d.set_zlim(0, 7)
ax3d.set_xlabel('X (m)', color='#7777bb', fontsize=8, labelpad=3)
ax3d.set_ylabel('Y (m)', color='#7777bb', fontsize=8, labelpad=3)
ax3d.set_zlabel('Z (m)', color='#7777bb', fontsize=8, labelpad=3)
ax3d.tick_params(colors='#555577', labelsize=7)
ax3d.xaxis.pane.fill=False; ax3d.yaxis.pane.fill=False; ax3d.zaxis.pane.fill=False
ax3d.xaxis.pane.set_edgecolor('#1a1a33')
ax3d.yaxis.pane.set_edgecolor('#1a1a33')
ax3d.zaxis.pane.set_edgecolor('#1a1a33')

# Ve waypoints
wx,wy,wz = WAYPOINTS[:,0],WAYPOINTS[:,1],WAYPOINTS[:,2]
ax3d.plot(wx,wy,wz,'o--',color='#ff8800',alpha=0.5,markersize=5,lw=1,label='Waypoints')
for i,(x,y,z) in enumerate(WAYPOINTS):
    ax3d.text(x,y,z+0.18,f'WP{i}',color='#ffaa44',fontsize=7)

# Ve san cat canh (grid nho)
grid_r=np.linspace(-0.5,0.5,5)
gx,gy=np.meshgrid(grid_r,grid_r)
ax3d.plot_surface(gx,gy,np.zeros_like(gx),alpha=0.08,color='#3333aa')

# Subplots ben phai
def make_sub(pos_gs, title, ylabel, ylim=None):
    ax = fig.add_subplot(pos_gs)
    ax.set_facecolor(PANEL)
    ax.tick_params(colors='#555577', labelsize=7)
    ax.set_title(title, color='#aabbdd', fontsize=8, pad=3)
    ax.set_ylabel(ylabel, color='#666688', fontsize=7)
    ax.grid(True, color=GRID, lw=0.5, ls=':')
    for sp in ax.spines.values(): sp.set_edgecolor('#1e1e3a')
    if ylim: ax.set_ylim(ylim)
    return ax

ax_alt = make_sub(gs[0,1], 'Altitude (m)', 'Z (m)')
ax_att = make_sub(gs[1,1], 'Roll / Pitch (deg)', 'deg', (-25,25))
ax_mot = make_sub(gs[2,1], 'Motor Pulses (us)', 'us', (980, 2020))
ax_mot.set_xlabel('t (s)', color='#555577', fontsize=7)

# Pre-plot altitude & attitude (tinh)
t_ax = np.arange(N) * DT
ax_alt.plot(t_ax, history_pos[:,2], color='#4d96ff', lw=1.2)
ax_alt.fill_between(t_ax, history_pos[:,2], alpha=0.15, color='#4d96ff')
ax_alt.axhline(0, color='#333355', lw=0.5)

ax_att.plot(t_ax, np.degrees(history_roll),  color='#ff6b6b', lw=1, label='Roll')
ax_att.plot(t_ax, np.degrees(history_pitch), color='#ffd93d', lw=1, label='Pitch')
ax_att.axhline(0, color='white', lw=0.4, ls='--', alpha=0.3)
ax_att.legend(fontsize=7, facecolor=PANEL, labelcolor='white', loc='upper right')

CLR_M = ['#ff6b6b','#ffd93d','#6bcb77','#4d96ff']
LBL_M = ['FR(M1)','FL(M2)','RR(M3)','RL(M4)']
for i in range(4):
    ax_mot.plot(t_ax, history_motor[:,i], color=CLR_M[i], lw=0.9, label=LBL_M[i], alpha=0.85)
ax_mot.legend(fontsize=6.5, facecolor=PANEL, labelcolor='white', ncol=2, loc='upper left')

# Duong vertical theo doi thoi gian hien tai
vline_alt  = ax_alt.axvline(0, color='white', lw=0.8, alpha=0.5)
vline_att  = ax_att.axvline(0, color='white', lw=0.8, alpha=0.5)
vline_mot  = ax_mot.axvline(0, color='white', lw=0.8, alpha=0.5)

# ─────────────────────────────────────────────
# 7. OBJECTS ANIMATION
# ─────────────────────────────────────────────
trail,     = ax3d.plot([],[],[], '-',  color='#00aaff', lw=1.2, alpha=0.6, zorder=2)
arm_l      = [ax3d.plot([],[],[], '-', color='#e0e0ff', lw=2.5,
                         solid_capstyle='round')[0] for _ in range(2)]
motor_dots,= ax3d.plot([],[],[], 'o',  color='#ff3333', markersize=7,
                        markeredgecolor='#ff9999', zorder=5)
center_dot,= ax3d.plot([],[],[], 'o',  color='#00ff88', markersize=9,
                        markeredgecolor='white', zorder=6)
shadow,    = ax3d.plot([],[],[], 'o',  color='#222244', markersize=7, alpha=0.4)
dropline,  = ax3d.plot([],[],[], '--', color='#444466', lw=0.7, alpha=0.5)

# 4 vong motor (circles)
motor_rings = []
for i in range(4):
    ring, = ax3d.plot([],[],[], 'o', color=CLR_M[i], markersize=11, alpha=0.35, zorder=4)
    motor_rings.append(ring)

# Status box
status_txt = ax3d.text2D(
    0.02, 0.97, '', transform=ax3d.transAxes,
    color='white', fontsize=8, va='top',
    fontfamily='monospace',
    bbox=dict(boxstyle='round,pad=0.5', fc='#080820', ec='#3333aa', alpha=0.85))

ax3d.legend(loc='upper right', fontsize=7, facecolor=PANEL, labelcolor='white',
            edgecolor='#2a2a44')

# ─────────────────────────────────────────────
# 8. UPDATE FUNCTION
# ─────────────────────────────────────────────
SKIP = 2

def update(frame):
    idx = min(frame * SKIP, N-1)
    t   = idx * DT
    ph  = history_pos[:idx+1]
    cx,cy,cz = history_pos[idx]
    roll     = history_roll[idx]
    pitch    = history_pitch[idx]
    wp       = history_wp[min(idx, N-1)]
    motors   = history_motor[idx]

    # Trail
    trail.set_data(ph[:,0], ph[:,1])
    trail.set_3d_properties(ph[:,2])

    # Arms
    m = drone_arms(cx,cy,cz,roll,pitch)
    arm_l[0].set_data([m[0,0],cx,m[1,0]], [m[0,1],cy,m[1,1]])
    arm_l[0].set_3d_properties([m[0,2],cz,m[1,2]])
    arm_l[1].set_data([m[2,0],cx,m[3,0]], [m[2,1],cy,m[3,1]])
    arm_l[1].set_3d_properties([m[2,2],cz,m[3,2]])

    # Motor dots + rings
    motor_dots.set_data(m[:,0], m[:,1])
    motor_dots.set_3d_properties(m[:,2])
    for i,ring in enumerate(motor_rings):
        ring.set_data([m[i,0]], [m[i,1]])
        ring.set_3d_properties([m[i,2]])
        # Kich thuoc ring the hien suc motor
        ms = 8 + (motors[i] - 1000) / 120
        ring.set_markersize(np.clip(ms, 5, 18))

    center_dot.set_data([cx],[cy]); center_dot.set_3d_properties([cz])
    shadow.set_data([cx],[cy]);     shadow.set_3d_properties([0.01])
    dropline.set_data([cx,cx],[cy,cy]); dropline.set_3d_properties([cz,0])

    # Vertical time cursor
    vline_alt.set_xdata([t,t])
    vline_att.set_xdata([t,t])
    vline_mot.set_xdata([t,t])

    # Status
    tw = WAYPOINTS[min(wp, len(WAYPOINTS)-1)]
    r_d = np.degrees(roll); p_d = np.degrees(pitch)
    spd = np.linalg.norm(history_pos[idx] - history_pos[max(idx-1,0)]) / DT
    status_txt.set_text(
        f" t  = {t:5.1f} s\n"
        f" WP = {wp}  -> ({tw[0]:.0f},{tw[1]:.0f},{tw[2]:.0f})m\n"
        f" Pos= ({cx:.1f}, {cy:.1f}, {cz:.1f}) m\n"
        f" Spd= {spd:.1f} m/s\n"
        f" Roll={r_d:+5.1f}  Pitch={p_d:+5.1f} deg\n"
        f" M1={motors[0]:.0f} M2={motors[1]:.0f}\n"
        f" M3={motors[2]:.0f} M4={motors[3]:.0f} us"
    )

    return (trail, *arm_l, motor_dots, center_dot, shadow, dropline,
            *motor_rings, status_txt, vline_alt, vline_att, vline_mot)

# ─────────────────────────────────────────────
# 9. CHAY ANIMATION
# ─────────────────────────────────────────────
frames_total = (N + SKIP - 1) // SKIP
ani = animation.FuncAnimation(
    fig, update,
    frames=frames_total,
    interval=35,
    blit=False,
    repeat=True
)

print(f"[OK] {N} steps | {len(WAYPOINTS)} waypoints | {frames_total} frames")
print("[OK] Dong cua so de thoat.")
plt.show()
