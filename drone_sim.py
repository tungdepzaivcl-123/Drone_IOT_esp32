import numpy as np
import matplotlib.pyplot as plt

# 1. Thông số vật lý Drone (Giả lập F450)
Iyy = 0.005
dt = 0.004      # 250Hz
time = np.arange(0, 2.5 + dt, dt)

# 2. Thông số PID
Kp = 1.25
Ki = 4.0
Kd = 0.45

# 3. Khởi tạo
theta = 0.0
theta_dot = 0.0
target = 10.0
integral = 0.0
prev_error = 0.0
results = np.zeros(len(time))

# 4. Vòng lặp PID
for i in range(len(time)):
    error = target - theta

    integral += error * dt
    integral = max(min(integral, 15), -15)  # Anti-windup

    derivative = (error - prev_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative

    # Vật lý: Accel = Torque / Inertia
    theta_accel = output / Iyy
    theta_dot += theta_accel * dt
    theta += theta_dot * dt

    results[i] = theta
    prev_error = error

# 5. Vẽ đồ thị
fig, ax = plt.subplots(facecolor='white')
ax.plot(time, results, 'b', linewidth=2, label='Góc nghiêng thực tế')
ax.axhline(y=target, color='r', linestyle='--', linewidth=1.5, label='Mục tiêu (10 độ)')
ax.grid(True)
ax.set_xlabel('Thời gian (giây)')
ax.set_ylabel('Góc nghiêng (độ)')
ax.set_title(f'Mô phỏng Pitch Drone với Ki = {Ki}')
ax.legend()
plt.tight_layout()
plt.show()