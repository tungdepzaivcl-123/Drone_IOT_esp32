%% MÔ PHỎNG ĐIỀU KHIỂN PITCH DRONE (Bản Full)
clear all; clc; close all;

% 1. Thông số vật lý Drone (Giả lập F450)
Iyy = 0.005;    
dt = 0.004;     % 250Hz
time = 0:dt:2.5;

% 2. Thông số PID
Kp = 1.25; 
Ki = 4.0;  % Con số bạn đang thắc mắc
Kd = 0.45;

% 3. Khởi tạo
theta = 0; theta_dot = 0; target = 10;
integral = 0; prev_error = 0;
results = zeros(length(time), 1);

% 4. Vòng lặp PID
for i = 1:length(time)
    error = target - theta;
    
    integral = integral + (error * dt);
    integral = max(min(integral, 15), -15); % Anti-windup
    
    derivative = (error - prev_error) / dt;
    output = Kp*error + Ki*integral + Kd*derivative;
    
    % Vật lý: Accel = Torque / Inertia
    theta_accel = output / Iyy;
    theta_dot = theta_dot + theta_accel * dt;
    theta = theta + theta_dot * dt;
    
    results(i) = theta;
    prev_error = error;
end % <- PHẢI CÓ TỪ KHÓA END NÀY

% 5. Vẽ đồ thị
figure('Color', 'w');
plot(time, results, 'b', 'LineWidth', 2);
hold on;
yline(target, 'r--', 'Mục tiêu (10 độ)', 'LineWidth', 1.5);
grid on;
xlabel('Thời gian (giây)');
ylabel('Góc nghiêng (độ)');
title(['Mô phỏng Pitch Drone với Ki = ', num2str(Ki)]);