"""
Drone Hover Simulation for heavier frame (1.6kg)
================================================
- Mô phỏng đơn giản tính độ cao và khí động khi tăng throttle.
- Kiểm tra throttle cần thiết để hover với 1.6kg.
- So sánh với cấu hình firmware hiện tại (HOVER_THR=620 trên 1.5kg).
"""

import numpy as np
import matplotlib.pyplot as plt

# Firmware hiện tại tuning cho ~1.5kg hover
HOVER_THR_15KG = 620.0   # tương đương 1620us trên motor pulse scale [1000..2000]
MAX_THR = 1000.0          # tương đương full throttle 2000us trong firmware
MIN_IDLE = 260.0
MASS_15KG = 1.5
MASS_16KG = 1.6
GRAVITY = 9.81

# Thrust model: assume scale linear với motor command
# Lực đẩy hover cho 1.5kg = MASS_15KG * g, tương đương HOVER_THR_15KG.
# Thrust (N) = k * thr_cmd. Tính k từ 1.5kg hover.
THRUST_PER_UNIT = MASS_15KG * GRAVITY / HOVER_THR_15KG

# Sim parameters
DT = 0.01
SIM_TIME = 5.0
THROTTLE_RC = np.array([1300, 1460, 1540, 1620, 1660, 1700, 1800])


def rc_to_thrust_unit(rc):
    thr = np.clip((rc - 1000) / 1000.0 * MAX_THR, 0, MAX_THR)
    if thr < MIN_IDLE:
        thr = MIN_IDLE
    return thr


def simulate_height(rc_throttle, mass):
    steps = int(SIM_TIME / DT)
    z = np.zeros(steps)
    vz = np.zeros(steps)
    thr = rc_to_thrust_unit(rc_throttle)
    thrust = thr * THRUST_PER_UNIT
    for i in range(1, steps):
        force_net = thrust - mass * GRAVITY - 0.70 * vz[i-1] * abs(vz[i-1])
        az = force_net / mass
        vz[i] = vz[i-1] + az * DT
        z[i] = max(z[i-1] + vz[i] * DT, 0.0)
    return z, vz, thrust, thr


if __name__ == "__main__":
    print("=== Đánh giá hover throttle cho 1.6kg ===")
    print(f"Thrust per unit = {THRUST_PER_UNIT:.3f} N/unit")
    print(f"Hover thrust 1.5kg = {MASS_15KG*GRAVITY:.2f} N")
    print(f"Hover thrust 1.6kg = {MASS_16KG*GRAVITY:.2f} N")
    print()

    fig, axs = plt.subplots(2, 1, figsize=(10, 9))

    for rc in THROTTLE_RC:
        z, vz, thrust, thr_cmd = simulate_height(rc, MASS_16KG)
        print(f"RC={rc} -> thr_cmd={thr_cmd:.1f}, thrust={thrust:.1f} N, z_final={z[-1]:.2f} m")
        axs[0].plot(np.linspace(0, SIM_TIME, len(z)), z, label=f"RC {rc}")
        axs[1].plot(np.linspace(0, SIM_TIME, len(vz)), vz, label=f"RC {rc}")

    axs[0].set_title('Height vs Time for 1.6kg')
    axs[0].set_ylabel('Height (m)')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].set_title('Vertical speed vs Time for 1.6kg')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Vz (m/s)')
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()

    # Tính throttle hover dự kiến cho 1.6kg
    hover_thr_16kg = HOVER_THR_15KG * (MASS_16KG / MASS_15KG)
    hover_rc = 1000 + hover_thr_16kg
    print() 
    print(f"Dự kiến hover throttle cho 1.6kg: {hover_thr_16kg:.1f} unit (~{hover_rc:.0f} RC)")
    print("Lưu ý: Nếu RC=1300 thì không đủ lực để hover với 1.6kg.")
