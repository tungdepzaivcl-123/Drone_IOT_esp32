%% MÔ PHỎNG ĐIỀU KHIỂN PITCH DRONE
clear all; clc;

% 1. Thông số vật lý Drone (Giả lập F450)
Iyy = 0.005;    % Mô-men quán tính trục Pitch
dt = 0.004;     % Looptime 250Hz (y hệt code ESP32 của bạn)
time = 0:dt:2;  % Mô phỏng trong 2 giây

% 2. Thông số PID (Thử nghiệm con số bạn hỏi)
Kp = 1.2; 
Ki = 4.0;  % Con số 4.0 bạn muốn thử
Kd = 0.45;

% 3. Khởi tạo biến
theta = 0;      % Góc hiện tại
theta_dot = 0;  % Vận tốc góc
target = 10;    % Mục tiêu: nghiêng 10 độ
integral = 0;
prev_error = 0;

% Mảng lưu kết quả để vẽ đồ thị
results = zeros(length(time), 1);

% 4. Vòng lặp PID (Y hệt logic trong hàm calculatePID của ESP32)
for i = 1:length(time)
    error = target - theta;
    
    % Tính Integral (I-term)
    integral = integral + (error * dt);
    % Giới hạn I-term (Anti-windup) - Cực kỳ quan trọng với Ki lớn
    integral = max(min(integral, 10), -10); 
    
    % Tính Derivative (D-term)simulink
    derivative = (error - prev_error) / dt;
    
    % Tổng PID (Lực mô-men đầu ra)
    output = Kp*error + Ki*integral + Kd*derivative;
    
    % Mô phỏng vật lý (Gia tốc = Lực / Quán tính)
    theta_acceleration = output / Iyy;
    theta_dot = theta_dot + theta_acceleration * dt;
    theta = theta + theta_dot * dt;
    
    % Lưu dữ liệu
    results(i) = theta;
    prev_error = error;
end

% 5. Vẽ đồ thị kết quả
plot(time, results, 'LineWidth', 2);
hold on;
yline(target, '--r', 'Mục tiêu 10 độ');
grid on;
title(['Mô phỏng PID với Ki = ', num2str(Ki)]);
xlabel('Thời gian (s)');
ylabel('Góc (độ)');