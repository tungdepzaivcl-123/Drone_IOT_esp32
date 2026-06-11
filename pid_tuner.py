"""
PHÂN TÍCH NHANH - Mixer + PID kiểm tra thực tế
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# VẬT LÝ
L=0.225; Ixx=0.013; K_T=1.275e-5; K_Q=0.016; D=0.003
DT=0.004; FREQ=250; MAX_I=75; MAX_THR=1000; AG=0.2

def T(m): F=K_T*m**2; return (F[0]+F[2]-F[1]-F[3])*L, (F[0]+F[1]-F[2]-F[3])*L, (F[0]+F[3]-F[1]-F[2])*K_Q
def mx_old(t,r,p,y): return np.clip([t+r+p+y,t-r+p-y,t+r-p-y,t-r-p+y],0,MAX_THR)
def mx_new(t,r,p,y): return np.clip([t-p-r-y,t-p+r+y,t+p-r+y,t+p+r-y],0,MAX_THR)

# Kiểm tra mixer
h=500; pid=-50.0
for name,fn in [("Mixer CŨ",mx_old),("Mixer MỚI",mx_new)]:
    m=fn(h,pid,0,0); tau_r=T(np.array(m,float))[0]
    print(f"{name}: pid_r=-50 → tau_roll={tau_r*1000:.1f}mN·m → {'✅ ĐÚNG' if tau_r>0 else '❌ SAI'}")

# Sim 1 trục
def sim(kp,ki,kd,ad,mfn,target=50.0,t_end=4.0):
    n=int(t_end/DT); w=sg=integ=pe=deriv=0.0; hv=500
    t_arr,g_arr,pid_arr=[],[],[]
    for i in range(n):
        t=i*DT; cmd=target if t>=0.3 else 0.0
        sg=AG*np.degrees(w)+(1-AG)*sg
        e=cmd-sg; integ=np.clip(integ+e*DT,-MAX_I,MAX_I)
        rd=(e-pe)*FREQ; deriv=(1-ad)*deriv+ad*rd
        p=np.clip(kp*e+ki*integ+kd*deriv,-400,400); pe=e
        m=np.array(mfn(hv,p,0,0),float)
        tr,_,_=T(m); w+=(tr-D*w)/Ixx*DT
        t_arr.append(t); g_arr.append(sg); pid_arr.append(p)
    return np.array(t_arr),np.array(g_arr),np.array(pid_arr)

DARK='#0a0a18'; PANEL='#10101e'
fig=plt.figure(figsize=(15,9),facecolor=DARK)
fig.suptitle('🚁  PID Analysis — Drone 1.3kg | Rate Mode 250Hz',color='white',fontsize=13,y=0.99)
gs=gridspec.GridSpec(2,3,figure=fig,hspace=0.45,wspace=0.3,left=0.07,right=0.97,top=0.93,bottom=0.08)

def ax_style(ax,title,yl):
    ax.set_facecolor(PANEL); ax.tick_params(colors='#555577',labelsize=8)
    ax.set_title(title,color='#ccddee',fontsize=9,pad=4)
    ax.set_ylabel(yl,color='#888',fontsize=8); ax.set_xlabel('t(s)',color='#888',fontsize=8)
    ax.grid(True,color='#181830',lw=0.5,ls=':')
    for s in ax.spines.values(): s.set_edgecolor('#1e1e3a')

TARGET=50.0

# -- 6 scenarios để hiểu vấn đề --
cases = [
    # (label, kp, ki, kd, alpha_d, mixer_fn, color, ls)
    ("Current (Ki=1.2, αd=0.012, MixerNEW)",   0.80, 1.20, 0.150, 0.012, mx_new, '#ff6b6b', '--'),
    ("Fix αd only (αd=0.05)",                   0.80, 1.20, 0.150, 0.050, mx_new, '#ff9944', '-.'),
    ("Fix Ki+αd (Ki=0.5, αd=0.05)",             0.80, 0.50, 0.150, 0.050, mx_new, '#ffd93d', ':'),
    ("Tuned NEW (Kp=0.6,Ki=0.3,Kd=0.2,αd=0.05)", 0.60,0.30,0.200,0.050, mx_new, '#00ffcc', '-'),
    ("Tuned OLD mixer (same params)",            0.60, 0.30, 0.200, 0.050, mx_old, '#4d96ff', '-'),
    ("Conservative start (Kp=0.4,Ki=0,Kd=0.1)", 0.40, 0.00, 0.100, 0.05, mx_new, '#aaffaa', '-'),
]

ax0=fig.add_subplot(gs[0,:]); ax_style(ax0,'📈 Step Response Roll Rate — 6 scenarios','°/s')
ax0.axhline(TARGET,color='white',lw=1,ls='--',alpha=0.4,label=f'Target {TARGET}°/s')
ax0.axhspan(TARGET*0.95,TARGET*1.05,alpha=0.05,color='lime')
ax0.set_xlim(0,4); ax0.set_ylim(-30,100)

ax1=fig.add_subplot(gs[1,0]); ax_style(ax1,'⚙️ PID Output','units')
ax2=fig.add_subplot(gs[1,1]); ax_style(ax2,'⚡ Ki ảnh hưởng thế nào','°/s')
ax3=fig.add_subplot(gs[1,2]); ax_style(ax3,'🔬 αd ảnh hưởng thế nào','°/s')

ax2.axhline(TARGET,color='white',lw=0.8,ls='--',alpha=0.4)
ax3.axhline(TARGET,color='white',lw=0.8,ls='--',alpha=0.4)

for nm,kp,ki,kd,ad,mfn,clr,ls in cases:
    t,g,pid=sim(kp,ki,kd,ad,mfn)
    os_val=max(0,g.max()-TARGET)
    ax0.plot(t,g,color=clr,lw=1.6,ls=ls,label=f"{nm}  OS={os_val:.0f}°/s")
    ax1.plot(t,pid,color=clr,lw=1.2,ls=ls)

ax0.legend(fontsize=7.5,facecolor=PANEL,labelcolor='white',loc='upper left')

# Ki effect
for ki_v,clr in zip([0.0,0.3,0.6,1.2],['#aaffaa','#00ffcc','#ffd93d','#ff6b6b']):
    t,g,_=sim(0.6,ki_v,0.2,0.05,mx_new)
    ax2.plot(t,g,color=clr,lw=1.4,label=f'Ki={ki_v}')
ax2.legend(fontsize=8,facecolor=PANEL,labelcolor='white')
ax2.set_xlim(0,4)

# alpha_d effect
for ad_v,clr in zip([0.012,0.025,0.05,0.1],['#ff6b6b','#ffd93d','#00ffcc','#4d96ff']):
    t,g,_=sim(0.6,0.3,0.2,ad_v,mx_new)
    ax3.plot(t,g,color=clr,lw=1.4,label=f'αd={ad_v}')
ax3.legend(fontsize=8,facecolor=PANEL,labelcolor='white')
ax3.set_xlim(0,4)

plt.savefig('pid_tuning_result.png',dpi=120,bbox_inches='tight',facecolor=DARK)
print("✅ Done! Saved: pid_tuning_result.png")

# Print recommendations
print("\n"+"="*55)
print("KẾT QUẢ & KHUYẾN NGHỊ")
print("="*55)
for nm,kp,ki,kd,ad,mfn,clr,ls in cases:
    t,g,_=sim(kp,ki,kd,ad,mfn)
    os_v=max(0,g.max()-TARGET)
    g_final=g[-100:].mean()
    err_ss=abs(g_final-TARGET)
    print(f"{nm[:45]:45s}  OS={os_v:5.1f}°/s  SS_err={err_ss:5.1f}°/s")

plt.show()
