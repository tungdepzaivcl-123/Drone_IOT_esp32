import numpy as np
import matplotlib.pyplot as plt
import sys

# --- Thông số Vật lý Drone (Trục Roll) ---
MASS = 1.3 # kg
L = 0.22 # Chiều dài sải tay (m) F450
I_xx = 0.015 # Moment quán tính (kg*m^2)
MAX_THRUST_MOTOR = 8.0 # Lực đẩy dòng tối đa khoảng 800g mỗi motor (N)
MAX_PID = 400.0

# HoangViet params: kp_rate, ki_rate, kd_rate, kp_angle
HOANGVIET_PARAMS = [0.700993, 1.265000, 0.112501, 2.0]

def constrain(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def run_simulation(params, target_angle=15.0, return_history=False):
    kp_rate, ki_rate, kd_rate, kp_angle = params
    
    dt = 0.004 # Tần số tính toán 250Hz khớp code ESP32
    t_end = 2.0 # Chạy mô phỏng 2 giây
    steps = int(t_end / dt)
    
    angle = 0.0
    rate = 0.0
    
    integral = 0.0
    prev_error = 0.0
    derivative = 0.0
    alpha_d = 0.012
    
    error_sum_sq = 0.0
    
    history_t = []
    history_angle = []
    history_rate = []
    history_motor = []

    for step in range(steps):
        # 1. Outer loop (Bộ cân bằng Góc - Angle Mode)
        target_rate = kp_angle * (target_angle - angle)
        
        # 2. Inner loop (Bộ khóa tốc Vận Tốc - Rate Mode)
        error = target_rate - rate
        
        integral = constrain(integral + error * dt, -75.0, 75.0)
        
        raw_d = (error - prev_error) / dt
        derivative = (1.0 - alpha_d) * derivative + alpha_d * raw_d
        
        pid_out = constrain(kp_rate * error + ki_rate * integral + kd_rate * derivative, -MAX_PID, MAX_PID)
        prev_error = error
        
        # 3. Mixer phi tuyến & Động lực lực Cứng (Rigid Body Dynamics)
        # Giả lập Moment xoắn sinh ra bởi chênh lệch biến PID
        # pid_out = 400 -> motor lật thay đổi 100% lực đẩy
        torque = (pid_out / 400.0) * MAX_THRUST_MOTOR * L
        
        # 4. Gió tạt mạnh ở mốc 0.5s -> 0.6s
        if step > int(0.5/dt) and step < int(0.6/dt):
            torque += 0.5 # Gió tạt làm nghiêng drone
            
        angular_accel = torque / I_xx
        
        rate += angular_accel * dt
        angle += rate * dt
        
        # Hàm Cost: Phạt lỗi góc (càng chênh lệch 15 độ càng nặng) và dao động
        error_sum_sq += (target_angle - angle)**2 + 0.5 * (rate)**2
        
        if return_history:
            history_t.append(step * dt)
            history_angle.append(angle)
            history_rate.append(rate)
            history_motor.append(pid_out)
            
    if return_history:
        return history_t, history_angle, history_rate, history_motor
    return error_sum_sq

def twiddle():
    print("🤖 Đang chạy thuật toán Twiddle để tìm PID tối ưu...")
    params = [0.8, 1.0, 0.1, 2.5] # Dự đoán khởi điểm
    d_params = [0.1, 0.1, 0.02, 0.2]
    best_error = run_simulation(params)
    
    iteration = 0
    while sum(d_params) > 0.001 and iteration < 100:
        for i in range(len(params)):
            params[i] += d_params[i]
            err = run_simulation(params)
            
            if err < best_error:
                best_error = err
                d_params[i] *= 1.1
            else:
                params[i] -= 2 * d_params[i]
                err = run_simulation(params)
                if err < best_error:
                    best_error = err
                    d_params[i] *= 1.1
                else:
                    params[i] += d_params[i]
                    d_params[i] *= 0.9
        iteration += 1
    print(f"✨ Twiddle hoàn tất sau {iteration} epoch! Cost: {best_error:.2f}")
    return params

def plot_sim():
    print("Trích xuất thông số gốc của QUAD_KH_HOANGVIET...")
    t, a_hv, r_hv, m_hv = run_simulation(HOANGVIET_PARAMS, return_history=True)
    
    best_params = twiddle()
    print(f"🥇 Params Tối ưu (Dựa trên Twiddle):")
    print(f"   kp_inner = {best_params[0]:.4f}")
    print(f"   ki_inner = {best_params[1]:.4f}")
    print(f"   kd_inner = {best_params[2]:.4f}")
    print(f"   kp_angle = {best_params[3]:.4f}")
    
    t, a_tw, r_tw, m_tw = run_simulation(best_params, return_history=True)
    
    plt.style.use('dark_background')
    plt.figure(figsize=(12, 8))
    
    # Biểu đồ Góc nghiêng
    plt.subplot(2, 1, 1)
    plt.plot(t, a_hv, label=f'QUAD_KH_HOANGVIET (Kp={HOANGVIET_PARAMS[0]:.2f})', color='#ff6b6b', linewidth=2)
    plt.plot(t, a_tw, label=f'Twiddle AI Optimized', color='#4ecdc4', linestyle='--', linewidth=2.5)
    plt.axhline(y=15.0, color='yellow', linestyle=':', label='Target Angle (Nghiêng 15 độ)')
    
    # Chỉ định bão gió
    plt.axvspan(0.5, 0.6, color='white', alpha=0.1)
    plt.text(0.55, 5, 'Gió tạt mạnh', rotation=90, color='white')
    
    plt.title('Mô phỏng Bay Cân Bằng (Angle Mode) - Frame 1.3Kg (Phản ứng Gió)', fontsize=14)
    plt.ylabel('Angle (Độ nghiêng)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    
    # Biểu đồ Motor/PID
    plt.subplot(2, 1, 2)
    plt.plot(t, m_hv, label='Motor Output (HoangViet)', color='#ff6b6b')
    plt.plot(t, m_tw, label='Motor Output (Twiddle)', color='#4ecdc4', linestyle='--')
    plt.xlabel('Thời gian (Giây)', fontsize=12)
    plt.ylabel('PID Output Bù động cơ (0-400)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    plot_sim()
