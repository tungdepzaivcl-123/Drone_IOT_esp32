import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ─────────────────────────────────────────────
# 1. THÔNG SỐ VẬT LÝ & NHIỄU (Dựa trên thông số thực tế của Tùng)
# ─────────────────────────────────────────────
MASS = 1.3                 # kg
GRAVITY = 9.81             # m/s^2
MAX_THRUST = 32.0          # Lực đẩy tối đa (N)
DRAG = 0.70                # Hệ số cản

# Các nguồn nhiễu thực tế
MOTOR_JITTER_STD = 0.5     # Rung động cơ khí (N)
WIND_VERTICAL_STD = 1.2    # Gió giật lên xuống (N)
SENSOR_NOISE_STD = 0.15    # Sai số cảm biến Baro/MPU (m)
BATTERY_EFFICIENCY = 0.96  # Sụt áp pin 4% khi tải nặng

# ─────────────────────────────────────────────
# 2. CẤU HÌNH MÔ PHỎNG
# ─────────────────────────────────────────────
DT = 0.01 
SIM_TIME = 5.0
THROTTLE_START = 1000      # 1000us (Min)
THROTTLE_MAX = 2000        # 2000us (Max)
TIME_TO_MAX_THROTTLE = 2.0 # Thời gian đẩy cần ga từ 0 lên 100%

def throttle_to_thrust(throttle):
    thr_normalized = (throttle - 1000) / 1000.0
    return np.clip(thr_normalized * MAX_THRUST * BATTERY_EFFICIENCY, 0, MAX_THRUST)

# ─────────────────────────────────────────────
# 3. VÒNG LẶP MÔ PHỎNG (SIMULATION LOOP)
# ─────────────────────────────────────────────
def simulate_takeoff_noisy():
    time_steps = int(SIM_TIME / DT)
    times = np.linspace(0, SIM_TIME, time_steps)
    
    true_pos_x = np.zeros(time_steps)  # X = 0
    true_pos_y = np.zeros(time_steps)  # Y = 0
    true_pos_z = np.zeros(time_steps)  # Độ cao thực tế
    true_vel_z = np.zeros(time_steps)  # Vận tốc thực tế
    measured_pos_z = np.zeros(time_steps) # Dữ liệu ESP32 nhận được
    
    thrusts = np.zeros(time_steps)
    throttles = np.zeros(time_steps)

    for i in range(1, time_steps):
        t = times[i]

        # Mô phỏng hành động đẩy cần ga của phi công
        if t < TIME_TO_MAX_THROTTLE:
            throttle = THROTTLE_START + (THROTTLE_MAX - THROTTLE_START) * (t / TIME_TO_MAX_THROTTLE)
        else:
            throttle = THROTTLE_MAX
        throttles[i] = throttle

        # 1. Tính lực đẩy có nhiễu motor
        base_thrust = throttle_to_thrust(throttle)
        actual_thrust = base_thrust + np.random.normal(0, MOTOR_JITTER_STD)
        thrusts[i] = actual_thrust

        # 2. Gió giật bất ngờ
        wind_force = np.random.normal(0, WIND_VERTICAL_STD)

        # 3. Tổng hợp lực (F_net = Thrust - Weight - Drag + Wind)
        force_net = actual_thrust - (MASS * GRAVITY) - (DRAG * true_vel_z[i-1]) + wind_force
        accel_z = force_net / MASS

        # 4. Tích phân cập nhật trạng thái
        true_vel_z[i] = true_vel_z[i-1] + accel_z * DT
        true_pos_z[i] = true_pos_z[i-1] + true_vel_z[i] * DT

        # Xử lý va chạm mặt đất
        if true_pos_z[i] < 0:
            true_pos_z[i] = 0
            true_vel_z[i] = 0

        # 5. Giả lập dữ liệu cảm biến (Sensor Noise)
        measured_pos_z[i] = true_pos_z[i] + np.random.normal(0, SENSOR_NOISE_STD)

    return times, true_pos_x, true_pos_y, true_pos_z, measured_pos_z, true_vel_z, thrusts, throttles

# ─────────────────────────────────────────────
# 4. TRỰC QUAN HÓA DỮ LIỆU
# ─────────────────────────────────────────────
times, true_x, true_y, true_z, meas_z, vel_z, thrusts, throttles = simulate_takeoff_noisy()

fig, axs = plt.subplots(3, 1, figsize=(10, 12))

# Biểu đồ 1: Độ cao - Sự khác biệt giữa thực tế và cảm biến
axs[0].plot(times, true_z, label='Độ cao Thật (Ground Truth)', color='#1f77b4', linewidth=3)
axs[0].scatter(times[::5], meas_z[::5], label='Dữ liệu Cảm biến (Noisy)', color='#ff7f0e', s=10, alpha=0.6)
axs[0].set_title('PHÂN TÍCH ĐỘ CAO CẤT CÁNH', fontsize=14, fontweight='bold')
axs[0].set_ylabel('Độ cao (m)')
axs[0].legend()
axs[0].grid(True, linestyle='--', alpha=0.7)

# Biểu đồ 2: Vận tốc Z - Tác động của gió và rung động
axs[1].plot(times, vel_z, label='Vận tốc Z (m/s)', color='#2ca02c', linewidth=2)
axs[1].set_title('ĐỘ BIẾN THIÊN VẬN TỐC', fontsize=12)
axs[1].set_ylabel('Vận tốc (m/s)')
axs[1].grid(True, linestyle='--', alpha=0.7)
axs[1].legend()

# Biểu đồ 3: Lực đẩy vs Trọng lượng
axs[2].plot(times, thrusts, label='Lực đẩy thực tế (có nhiễu)', color='#d62728', alpha=0.8)
axs[2].axhline(y=MASS*GRAVITY, color='#000000', linestyle='--', label='Ngưỡng cất cánh (Weight)')
axs[2].set_title('PHÂN TÍCH LỰC ĐẨY MOTOR', fontsize=12)
axs[2].set_xlabel('Thời gian (s)')
axs[2].set_ylabel('Lực (N)')
axs[2].legend()
axs[2].grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.show()

# Vẽ mô hình 3D
fig3d = plt.figure(figsize=(8, 6))
ax3d = fig3d.add_subplot(111, projection='3d')
ax3d.plot(true_x, true_y, true_z, label='Trajectory', color='blue', linewidth=2)
ax3d.scatter(true_x[-1], true_y[-1], true_z[-1], color='red', s=50, label='Final Position')
ax3d.set_xlabel('X (m)')
ax3d.set_ylabel('Y (m)')
ax3d.set_zlabel('Z (m)')
ax3d.set_title('Mô phỏng 3D Cất cánh Drone (có nhiễu)')
ax3d.legend()
ax3d.grid(True)
plt.show()