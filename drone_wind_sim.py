"""
SIM DRONE + GIO GIAT (Wind Gust Test)
======================================
Kiem tra do on dinh PID khi co gio giat bat ngo.
Drone 1.3kg | 9 Waypoints | Gio ngau nhien
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import warnings
warnings.filterwarnings('ignore')

rng = np.random.default_rng(7)

# ─────────────────────────────────────────────
# 1. THONG SO VAT LY
# ─────────────────────────────────────────────
DT         = 0.05
MASS       = 1.3
GRAVITY    = 9.81
MAX_THRUST = 35.0
MAX_TILT   = 0.30
DRAG       = 0.70
MAX_VEL_XY = 2.5
MAX_VEL_Z  = 1.5

KP_XY = 0.40;  KI_XY = 0.010;  KD_XY = 0.55   # KD = 0.55 dang test
KP_Z  = 1.60;  KI_Z  = 0.030;  KD_Z  = 1.00

# ─────────────────────────────────────────────
# 2. GIO GIAT (Random Wind Gusts)
# ─────────────────────────────────────────────
WIND_STEADY   = np.array([0.6, 0.3, 0.0])   # N - gio lien tuc
WIND_GUST_MAX = 4.5    # N - suc giat toi da (kha manh voi drone 1.3kg)
WIND_GUST_TAU = 0.8    # s - time constant gust
GUST_PROB     = 0.03   # xac suat xuat hien gust moi buoc (3%)
GUST_DUR_MAX  = 25     # buoc gust toi da (~1.25s @ 50ms)

# ─────────────────────────────────────────────
# 3. WAYPOINTS
# ─────────────────────────────────────────────
WAYPOINTS = np.array([
    [ 0,  0,  0], [ 0,  0,  3], [ 5,  0,  3],
    [ 5,  5,  4], [ 0,  5,  4], [ 0,  0,  4],
    [ 2.5,2.5,5.5], [ 0, 0, 1], [ 0,  0,  0],
], dtype=float)

REACH_RADIUS = 0.25

# ─────────────────────────────────────────────
# 4. PID CLASS
# ─────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, i_lim=None):
        self.kp=kp; self.ki=ki; self.kd=kd
        self.i_lim=i_lim; self.I=0.0; self.pe=0.0
    def update(self, err, dt):
        self.I += err*dt
        if self.i_lim: self.I = np.clip(self.I, -self.i_lim, self.i_lim)
        D = (err - self.pe) / dt; self.pe = err
        return self.kp*err + self.ki*self.I + self.kd*D

pid_x = PID(KP_XY, KI_XY, KD_XY, i_lim=5)
pid_y = PID(KP_XY, KI_XY, KD_XY, i_lim=5)
pid_z = PID(KP_Z,  KI_Z,  KD_Z,  i_lim=8)

# ─────────────────────────────────────────────
# 5. SIM LOOP VOI GIO
# ─────────────────────────────────────────────
pos    = WAYPOINTS[0].copy().astype(float)
vel    = np.zeros(3)
wp_idx = 1

wind_gust    = np.zeros(3)
gust_counter = 0

hist_pos   = [pos.copy()]
hist_wp    = [1]
hist_roll  = [0.0]
hist_pitch = [0.0]
hist_motor = [np.ones(4)*1000.0]
hist_wind  = [np.zeros(3)]
hist_gust_active = [False]

MAX_STEPS = 5000
HOVER_THR = 1560

for step in range(MAX_STEPS):
    target = WAYPOINTS[wp_idx]
    err    = target - pos

    if np.linalg.norm(err) < REACH_RADIUS:
        if wp_idx < len(WAYPOINTS)-1:
            wp_idx += 1
        else:
            for _ in range(40):
                hist_pos.append(pos.copy())
                hist_wp.append(wp_idx)
                hist_roll.append(0.0); hist_pitch.append(0.0)
                hist_motor.append(np.ones(4)*HOVER_THR)
                hist_wind.append(np.zeros(3))
                hist_gust_active.append(False)
            break

    # --- Gio giat ngau nhien ---
    gust_now = False
    if gust_counter > 0:
        gust_counter -= 1
        gust_now = True
    elif rng.random() < GUST_PROB:
        gust_dir   = rng.uniform(-1, 1, 3); gust_dir[2]=0
        gust_dir  /= (np.linalg.norm(gust_dir) + 1e-6)
        wind_gust  = gust_dir * rng.uniform(WIND_GUST_MAX*0.5, WIND_GUST_MAX)
        gust_counter = rng.integers(5, GUST_DUR_MAX)
        gust_now = True
    else:
        wind_gust *= 0.85   # tat dan

    wind_total = WIND_STEADY + wind_gust

    # --- PID ---
    ax_cmd = np.clip(pid_x.update(err[0], DT), -MAX_TILT*GRAVITY, MAX_TILT*GRAVITY)
    ay_cmd = np.clip(pid_y.update(err[1], DT), -MAX_TILT*GRAVITY, MAX_TILT*GRAVITY)
    az_cmd = pid_z.update(err[2], DT)

    thrust = np.clip((az_cmd + GRAVITY)*MASS, 0, MAX_THRUST)
    accel  = np.array([ax_cmd, ay_cmd, thrust/MASS - GRAVITY])
    accel += wind_total / MASS   # luc gio tac dong
    accel -= DRAG * vel

    vel += accel * DT
    vel[0] = np.clip(vel[0], -MAX_VEL_XY, MAX_VEL_XY)
    vel[1] = np.clip(vel[1], -MAX_VEL_XY, MAX_VEL_XY)
    vel[2] = np.clip(vel[2], -MAX_VEL_Z,  MAX_VEL_Z)
    pos   = pos + vel * DT
    pos[2] = max(pos[2], 0.0)

    pitch = np.arctan2(ax_cmd, GRAVITY)
    roll  = np.arctan2(-ay_cmd, GRAVITY)

    thr_unit = np.clip((thrust/MAX_THRUST)*1000, 40, 1000)
    r_p = np.degrees(roll)*1.2; p_p = np.degrees(pitch)*1.2
    motors = np.array([
        1000+np.clip(thr_unit+r_p+p_p, 0, 1000),
        1000+np.clip(thr_unit-r_p+p_p, 0, 1000),
        1000+np.clip(thr_unit+r_p-p_p, 0, 1000),
        1000+np.clip(thr_unit-r_p-p_p, 0, 1000),
    ])

    hist_pos.append(pos.copy()); hist_wp.append(wp_idx)
    hist_roll.append(roll); hist_pitch.append(pitch)
    hist_motor.append(motors); hist_wind.append(wind_total.copy())
    hist_gust_active.append(gust_now)

hist_pos   = np.array(hist_pos)
hist_roll  = np.array(hist_roll)
hist_pitch = np.array(hist_pitch)
hist_motor = np.array(hist_motor)
hist_wind  = np.array(hist_wind)
hist_gust  = np.array(hist_gust_active)
N = len(hist_pos)
t_arr = np.arange(N)*DT

n_gusts = np.sum(np.diff(hist_gust.astype(int)) > 0)
print(f"[SIM] {N} steps | {n_gusts} dot gio giat | Max pos=({hist_pos[:,0].max():.1f},{hist_pos[:,1].max():.1f})")

# ─────────────────────────────────────────────
# 6. DRONE ARMS HELPER
# ─────────────────────────────────────────────
ARM_LEN = 0.30
def drone_arms(cx, cy, cz, roll, pitch):
    ab = np.array([[ARM_LEN,0,0],[-ARM_LEN,0,0],[0,ARM_LEN,0],[0,-ARM_LEN,0]])
    cr,sr=np.cos(roll),np.sin(roll); cp,sp=np.cos(pitch),np.sin(pitch)
    Rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    return (Ry@Rx@ab.T).T + np.array([cx,cy,cz])

# ─────────────────────────────────────────────
# 7. FIGURE
# ─────────────────────────────────────────────
DARK='#080812'; PANEL='#0e0e1e'; GRID='#181830'
CLR_M=['#ff6b6b','#ffd93d','#6bcb77','#4d96ff']

fig = plt.figure(figsize=(17, 9), facecolor=DARK)
fig.suptitle(
    f'WIND GUST TEST  |  Drone 1.3kg  |  Kd_pos={KD_XY}  |  Max Gust={WIND_GUST_MAX}N  |  {n_gusts} gusts',
    color='#ccddff', fontsize=12, y=0.99, fontweight='bold')

gs = gridspec.GridSpec(4, 2, figure=fig,
                       left=0.03, right=0.98, top=0.95, bottom=0.05,
                       hspace=0.55, wspace=0.08, width_ratios=[2.2,1])

ax3d = fig.add_subplot(gs[:,0], projection='3d')
ax3d.set_facecolor(PANEL)
ax3d.set_xlim(-1,6); ax3d.set_ylim(-1,6); ax3d.set_zlim(0,7)
ax3d.set_xlabel('X(m)',color='#7777bb',fontsize=8,labelpad=2)
ax3d.set_ylabel('Y(m)',color='#7777bb',fontsize=8,labelpad=2)
ax3d.set_zlabel('Z(m)',color='#7777bb',fontsize=8,labelpad=2)
ax3d.tick_params(colors='#555577',labelsize=7)
for p in [ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane]:
    p.fill=False; p.set_edgecolor('#1a1a33')

wx,wy,wz=WAYPOINTS[:,0],WAYPOINTS[:,1],WAYPOINTS[:,2]
ax3d.plot(wx,wy,wz,'o--',color='#ff8800',alpha=0.5,markersize=5,lw=1,label='Waypoints')
for i,(x,y,z) in enumerate(WAYPOINTS):
    ax3d.text(x,y,z+0.18,f'WP{i}',color='#ffaa44',fontsize=7)

def sub(pos_gs,title,yl,ylim=None):
    ax=fig.add_subplot(pos_gs)
    ax.set_facecolor(PANEL); ax.tick_params(colors='#555577',labelsize=7)
    ax.set_title(title,color='#aabbdd',fontsize=8,pad=3)
    ax.set_ylabel(yl,color='#666688',fontsize=7)
    ax.grid(True,color=GRID,lw=0.5,ls=':')
    for s in ax.spines.values(): s.set_edgecolor('#1e1e3a')
    if ylim: ax.set_ylim(ylim)
    return ax

ax_alt = sub(gs[0,1],'Altitude (m)','Z(m)')
ax_att = sub(gs[1,1],'Roll / Pitch (deg)','deg',(-25,25))
ax_mot = sub(gs[2,1],'Motor Pulses','us',(980,2020))
ax_wnd = sub(gs[3,1],'Wind Force (N)','N')
ax_wnd.set_xlabel('t(s)',color='#555577',fontsize=7)

# Static plots
ax_alt.plot(t_arr,hist_pos[:,2],color='#4d96ff',lw=1.2)
ax_alt.fill_between(t_arr,hist_pos[:,2],alpha=0.12,color='#4d96ff')
# Shade gust regions
in_g=False; sg=0
for i in range(N):
    if hist_gust[i] and not in_g:   in_g=True; sg=t_arr[i]
    elif not hist_gust[i] and in_g:
        ax_alt.axvspan(sg,t_arr[i],alpha=0.18,color='#ff4400')
        ax_att.axvspan(sg,t_arr[i],alpha=0.18,color='#ff4400')
        ax_mot.axvspan(sg,t_arr[i],alpha=0.12,color='#ff4400')
        ax_wnd.axvspan(sg,t_arr[i],alpha=0.18,color='#ff4400')
        in_g=False

ax_att.plot(t_arr,np.degrees(hist_roll), color='#ff6b6b',lw=1,label='Roll')
ax_att.plot(t_arr,np.degrees(hist_pitch),color='#ffd93d',lw=1,label='Pitch')
ax_att.axhline(0,color='white',lw=0.4,ls='--',alpha=0.3)
ax_att.legend(fontsize=7,facecolor=PANEL,labelcolor='white',loc='upper right')

for i in range(4):
    ax_mot.plot(t_arr,hist_motor[:,i],color=CLR_M[i],lw=0.9,alpha=0.85)

ax_wnd.plot(t_arr,hist_wind[:,0],color='#88aaff',lw=0.9,label='Wx')
ax_wnd.plot(t_arr,hist_wind[:,1],color='#aaffcc',lw=0.9,label='Wy')
ax_wnd.axhline(WIND_STEADY[0],color='#88aaff',lw=0.5,ls=':',alpha=0.4)
ax_wnd.legend(fontsize=7,facecolor=PANEL,labelcolor='white')

# Time cursors
vlines = [
    ax_alt.axvline(0,color='white',lw=0.8,alpha=0.5),
    ax_att.axvline(0,color='white',lw=0.8,alpha=0.5),
    ax_mot.axvline(0,color='white',lw=0.8,alpha=0.5),
    ax_wnd.axvline(0,color='white',lw=0.8,alpha=0.5),
]

# ─────────────────────────────────────────────
# 8. ANIMATION OBJECTS
# ─────────────────────────────────────────────
trail,     = ax3d.plot([],[],[],'b-',color='#00aaff',lw=1.2,alpha=0.6)
arm_l      = [ax3d.plot([],[],[],'-',color='#e0e0ff',lw=2.5,solid_capstyle='round')[0] for _ in range(2)]
mtr_dots,  = ax3d.plot([],[],[],'o',color='#ff3333',markersize=7,markeredgecolor='#ff9999',zorder=5)
ctr_dot,   = ax3d.plot([],[],[],'o',color='#00ff88',markersize=9,markeredgecolor='white',zorder=6)
shadow,    = ax3d.plot([],[],[],'o',color='#222244',markersize=7,alpha=0.4)
dropline,  = ax3d.plot([],[],[],'--',color='#444466',lw=0.7,alpha=0.5)

# Arrow gio (quiver)
gust_quiver = ax3d.quiver(0,0,0,0,0,0,color='#ff6600',linewidth=2,arrow_length_ratio=0.3)

motor_rings=[]
for i in range(4):
    r,=ax3d.plot([],[],[],'o',color=CLR_M[i],markersize=10,alpha=0.3,zorder=4)
    motor_rings.append(r)

status_txt = ax3d.text2D(0.02,0.97,'',transform=ax3d.transAxes,color='white',
    fontsize=8,va='top',fontfamily='monospace',
    bbox=dict(boxstyle='round,pad=0.5',fc='#080820',ec='#3333aa',alpha=0.85))

gust_lbl = ax3d.text2D(0.75,0.95,'',transform=ax3d.transAxes,
    color='#ff6600',fontsize=10,va='top',fontweight='bold')

ax3d.legend(loc='upper right',fontsize=7,facecolor=PANEL,labelcolor='white',edgecolor='#2a2a44')

# ─────────────────────────────────────────────
# 9. UPDATE
# ─────────────────────────────────────────────
SKIP=2
def update(frame):
    global gust_quiver
    idx=min(frame*SKIP, N-1); t=idx*DT
    ph=hist_pos[:idx+1]
    cx,cy,cz=hist_pos[idx]
    roll=hist_roll[idx]; pitch=hist_pitch[idx]
    wp=hist_wp[min(idx,N-1)]
    motors=hist_motor[idx]
    wind=hist_wind[idx]
    is_gust=hist_gust[idx]

    trail.set_data(ph[:,0],ph[:,1]); trail.set_3d_properties(ph[:,2])

    m=drone_arms(cx,cy,cz,roll,pitch)
    arm_l[0].set_data([m[0,0],cx,m[1,0]],[m[0,1],cy,m[1,1]]); arm_l[0].set_3d_properties([m[0,2],cz,m[1,2]])
    arm_l[1].set_data([m[2,0],cx,m[3,0]],[m[2,1],cy,m[3,1]]); arm_l[1].set_3d_properties([m[2,2],cz,m[3,2]])

    mtr_dots.set_data(m[:,0],m[:,1]); mtr_dots.set_3d_properties(m[:,2])
    ctr_dot.set_data([cx],[cy]);      ctr_dot.set_3d_properties([cz])
    shadow.set_data([cx],[cy]);       shadow.set_3d_properties([0.01])
    dropline.set_data([cx,cx],[cy,cy]); dropline.set_3d_properties([cz,0])

    for i,ring in enumerate(motor_rings):
        ring.set_data([m[i,0]],[m[i,1]]); ring.set_3d_properties([m[i,2]])
        ms=8+(motors[i]-1000)/120; ring.set_markersize(np.clip(ms,4,18))

    # Arrow gio
    gust_quiver.remove()
    wscale=0.15
    clr='#ff4400' if is_gust else '#886633'
    gust_quiver=ax3d.quiver(cx,cy,cz,
        wind[0]*wscale,wind[1]*wscale,0,
        color=clr,linewidth=2,arrow_length_ratio=0.4,alpha=0.9)

    for vl in vlines: vl.set_xdata([t,t])

    tw=WAYPOINTS[min(wp,len(WAYPOINTS)-1)]
    wf=np.linalg.norm(wind[:2])
    spd=np.linalg.norm(hist_pos[idx]-hist_pos[max(idx-1,0)])/DT
    status_txt.set_text(
        f" t   = {t:5.1f} s\n"
        f" WP  = {wp} -> ({tw[0]:.0f},{tw[1]:.0f},{tw[2]:.0f})m\n"
        f" Pos = ({cx:.1f},{cy:.1f},{cz:.1f}) m\n"
        f" Spd = {spd:.2f} m/s\n"
        f" Gio = {wf:.1f} N ({('GIAT!' if is_gust else 'binh thuong')})\n"
        f" M1={motors[0]:.0f} M2={motors[1]:.0f}\n"
        f" M3={motors[2]:.0f} M4={motors[3]:.0f} us"
    )
    gust_lbl.set_text("*** GIO GIAT! ***" if is_gust else "")

    return (trail,*arm_l,mtr_dots,ctr_dot,shadow,dropline,
            *motor_rings,status_txt,gust_lbl,*vlines)

SKIP=2
frames_total=(N+SKIP-1)//SKIP
ani=animation.FuncAnimation(fig,update,frames=frames_total,interval=35,blit=False,repeat=True)
print(f"[OK] {frames_total} frames | Dong cua so de thoat")
plt.show()
