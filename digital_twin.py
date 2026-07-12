import pygame
import requests
import math
import sys
import time
import threading

# =======================================================
# CẤU HÌNH IP CỦA DRONE
# Mặc định khi drone phát WiFi là 192.168.4.1
# =======================================================
DRONE_IP = "192.168.4.1"
URL = f"http://{DRONE_IP}/status"

# =======================================================
# KHỞI TẠO PYGAME
# =======================================================
pygame.init()
WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("🚁 Drone Digital Twin 2D")
clock = pygame.time.Clock()

font = pygame.font.SysFont("Segoe UI", 18)
font_bold = pygame.font.SysFont("Segoe UI", 20, bold=True)
font_large = pygame.font.SysFont("Segoe UI", 36, bold=True)

# Colors
DARK = (10, 14, 20)
CYAN = (0, 210, 255)
PURPLE = (168, 85, 247)
RED = (255, 68, 68)
GREEN = (0, 255, 136)
GRAY = (40, 50, 60)
LIGHT_GRAY = (100, 110, 120)
WHITE = (255, 255, 255)

# =======================================================
# TRẠNG THÁI DRONE (State)
# =======================================================
class DroneState:
    def __init__(self):
        self.armed = False
        self.roll = 0.0
        self.pitch = 0.0
        self.m1 = 1000
        self.m2 = 1000
        self.m3 = 1000
        self.m4 = 1000
        self.alt = 0.0
        self.last_update = 0

state = DroneState()

# Thread-safe flag để thoát
running = True

# =======================================================
# FETCH DATA THREAD
# Dùng thread riêng để không làm giật màn hình Pygame
# =======================================================
def fetch_data_thread():
    global state, running
    while running:
        try:
            resp = requests.get(URL, timeout=0.3)
            if resp.status_code == 200:
                data = resp.json()
                state.armed = data.get("armed", False)
                state.roll = data.get("roll", 0.0)
                state.pitch = data.get("pitch", 0.0)
                state.m1 = data.get("m1", 1000)
                state.m2 = data.get("m2", 1000)
                state.m3 = data.get("m3", 1000)
                state.m4 = data.get("m4", 1000)
                state.alt = data.get("alt", 0.0)
                state.last_update = time.time()
        except requests.exceptions.RequestException:
            pass
        time.sleep(0.1) # Fetch 10 lần / giây

thread = threading.Thread(target=fetch_data_thread, daemon=True)
thread.start()

# =======================================================
# HÀM VẼ (DRAW FUNCTIONS)
# =======================================================
def draw_motor(x, y, label, pwm):
    # Khung vòng ngoài
    pygame.draw.circle(screen, GRAY, (x, y), 45, 3)
    
    # Tính phần trăm (1000 = 0%, 2000 = 100%)
    pct = max(0.0, min(1.0, (pwm - 1000) / 1000.0))
    
    # Màu chuyển từ Xanh (0) sang Đỏ (1)
    r = int(255 * pct)
    g = int(255 * (1 - pct))
    color = (r, g, 50)
    
    # Vẽ lõi motor theo giá trị PWM
    radius = int(43 * pct)
    if radius > 0:
        pygame.draw.circle(screen, color, (x, y), radius)
    
    # Nhãn tên motor
    lbl = font_bold.render(label, True, CYAN)
    screen.blit(lbl, (x - lbl.get_width()//2, y - 70))
    
    # Giá trị PWM
    val = font.render(f"{pwm} us", True, WHITE)
    screen.blit(val, (x - val.get_width()//2, y + 50))

def draw_horizon(x, y, radius, pitch, roll):
    # Khung đồng hồ
    pygame.draw.circle(screen, GRAY, (x, y), radius)
    pygame.draw.circle(screen, CYAN, (x, y), radius, 3)
    
    # Tạo mask cắt (Clip) hình tròn
    rect = pygame.Rect(x - radius, y - radius, radius*2, radius*2)
    screen.set_clip(rect)
    
    # Giới hạn pitch để không vẽ vọt ra ngoài quá xa
    # Giả sử 90 độ = radius
    pitch_offset = (pitch / 90.0) * radius
    
    # Chuyển roll sang radian
    angle_rad = math.radians(roll)
    
    # Tính toán đường chân trời (horizon line)
    # dx, dy là vector dọc theo đường chân trời
    dx = radius * 2 * math.cos(angle_rad)
    dy = radius * 2 * math.sin(angle_rad)
    
    # cx, cy là tâm của đường chân trời (bị dịch chuyển bởi pitch)
    # Chú ý: Pitch mũi ngóc lên -> đường chân trời tụt xuống
    cx = x + pitch_offset * math.sin(angle_rad)
    cy = y + pitch_offset * math.cos(angle_rad)
    
    p1 = (cx - dx, cy - dy)
    p2 = (cx + dx, cy + dy)
    
    # Vector pháp tuyến (để vẽ Bầu trời và Mặt đất)
    nx = dy * 2
    ny = -dx * 2
    
    # Vẽ Bầu trời (Xanh lơ)
    sky_p1 = (p1[0] + nx, p1[1] + ny)
    sky_p2 = (p2[0] + nx, p2[1] + ny)
    pygame.draw.polygon(screen, (40, 130, 210), [p1, p2, sky_p2, sky_p1])
    
    # Vẽ Mặt đất (Nâu đất)
    gnd_p1 = (p1[0] - nx, p1[1] - ny)
    gnd_p2 = (p2[0] - nx, p2[1] - ny)
    pygame.draw.polygon(screen, (139, 69, 19), [p1, p2, gnd_p2, gnd_p1])
    
    # Đường line chân trời trắng
    pygame.draw.line(screen, WHITE, p1, p2, 3)
    
    # Hủy Clip
    screen.set_clip(None)
    
    # Vẽ Crosshair (Biểu tượng cố định máy bay)
    pygame.draw.line(screen, RED, (x - 30, y), (x - 10, y), 4)
    pygame.draw.line(screen, RED, (x + 30, y), (x + 10, y), 4)
    pygame.draw.line(screen, RED, (x, y), (x, y + 15), 4)
    pygame.draw.circle(screen, RED, (x, y), 3)

# =======================================================
# MAIN LOOP
# =======================================================
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    screen.fill(DARK)
    now = time.time()
    
    # Header Title
    title = font_large.render("🚁 DRONE DIGITAL TWIN 2D", True, CYAN)
    screen.blit(title, (WIDTH//2 - title.get_width()//2, 20))
    
    # Connection Status
    is_connected = (now - state.last_update < 1.0)
    conn_color = GREEN if is_connected else RED
    conn_text = f"WiFi: CONNECTED ({DRONE_IP})" if is_connected else "WiFi: DISCONNECTED"
    conn_lbl = font_bold.render(conn_text, True, conn_color)
    screen.blit(conn_lbl, (30, 20))
    
    # Armed Status
    arm_color = RED if state.armed else GREEN
    arm_text = "⚠️ ARMED" if state.armed else "✅ DISARMED"
    arm_lbl = font_bold.render(arm_text, True, arm_color)
    screen.blit(arm_lbl, (30, 50))
    
    # Altitude
    alt_lbl = font_bold.render(f"Altitude: {state.alt:.2f} m", True, WHITE)
    screen.blit(alt_lbl, (30, 80))
    
    # ---------------------------------------------------
    # KHU VỰC 1: HÌNH ẢNH DRONE TOP-DOWN (Bên trái)
    # ---------------------------------------------------
    cx, cy = 300, 350
    span = 140
    
    # Vẽ các cánh tay (Arms)
    pygame.draw.line(screen, GRAY, (cx - span, cy - span), (cx + span, cy + span), 12)
    pygame.draw.line(screen, GRAY, (cx - span, cy + span), (cx + span, cy - span), 12)
    
    # Mũi tên chỉ hướng trước (Front)
    pygame.draw.polygon(screen, RED, [(cx, cy - 35), (cx - 15, cy - 10), (cx + 15, cy - 10)])
    
    # Thân Drone (Body)
    pygame.draw.circle(screen, DARK, (cx, cy), 35)
    pygame.draw.circle(screen, CYAN, (cx, cy), 35, 3)
    body_txt = font.render("BODY", True, WHITE)
    screen.blit(body_txt, (cx - body_txt.get_width()//2, cy - body_txt.get_height()//2))
    
    # Vẽ 4 Motors
    draw_motor(cx + span, cy - span, "M1 (FR)", state.m1)
    draw_motor(cx - span, cy - span, "M2 (FL)", state.m2)
    draw_motor(cx + span, cy + span, "M3 (RR)", state.m3)
    draw_motor(cx - span, cy + span, "M4 (RL)", state.m4)
    
    # ---------------------------------------------------
    # KHU VỰC 2: ARTIFICIAL HORIZON (Bên phải)
    # ---------------------------------------------------
    hx, hy = 700, 350
    hradius = 130
    draw_horizon(hx, hy, hradius, state.pitch, state.roll)
    
    p_lbl = font_bold.render(f"PITCH: {state.pitch:>6.1f}°", True, WHITE)
    r_lbl = font_bold.render(f"ROLL:  {state.roll:>6.1f}°", True, WHITE)
    screen.blit(p_lbl, (hx - p_lbl.get_width()//2, hy + hradius + 20))
    screen.blit(r_lbl, (hx - r_lbl.get_width()//2, hy + hradius + 50))
    
    # Mũi tên hướng dẫn Roll/Pitch
    # Chỉ số âm dương Pitch Roll
    info_txt = font.render("(Mũi chìm = Pitch +, Nghiêng trái = Roll +)", True, LIGHT_GRAY)
    screen.blit(info_txt, (hx - info_txt.get_width()//2, hy + hradius + 85))

    # Cập nhật màn hình
    pygame.display.flip()
    clock.tick(60) # 60 FPS

pygame.quit()
sys.exit()
