import serial
import json
import time
import sys
import threading
import math

PORT = "COM10"
BAUD = 115200

# Bắt đầu P nhỏ hơn (drone nhỏ nhạy hơn)
current_p = 0.05
history = []

print(f"[*] Dang ket noi toi {PORT}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f"[!] Loi: {e}")
    print("Vui long tat Serial Monitor tren Arduino IDE roi thu lai!")
    sys.exit(1)

print("=" * 50)
print("  AUTOTUNE ROLL (TRAI/PHAI) - V2.0")
print("=" * 50)
print("[OK] Da ket noi! Vui long ARM drone va tang ga nhe.")
print()
print("HUONG DAN:")
print(" + : Tang P len 0.05")
print(" - : Giam P xuong 0.05")
print(" a : Phan tich rung lac (4 giay)")
print(" s : Xem P hien tai va goc roll truc tiep")
print(" q : Thoat")
print()
print("CACH CAM DRONE DUNG CACH:")
print(" -> Dat drone tren san phang, GA THAP (1200-1300)")
print(" -> Nhan nhe vao 1 canh drone de kich rung, ROI BUC TAY RA")
print(" -> TUYET DOI KHONG CAM TAY VI TAY RUNG SE LAM SACH KET QUA")
print()

def read_serial():
    global history
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("{") and line.endswith("}"):
                data = json.loads(line)
                t = data['t']
                r = data['r']
                history.append((t, r))
                if len(history) > 500:  # 500 mau ~ 10 giay
                    history.pop(0)
            elif line and not line.startswith("{"):
                # In ra cac thong bao debug tu drone
                if "[" in line:
                    pass  # Bo qua debug log
        except Exception:
            pass

t = threading.Thread(target=read_serial, daemon=True)
t.start()

def show_realtime():
    """Hien thi goc roll truc tiep 3 giay"""
    print("\n[LIVE] Goc Roll trong 3 giay (+ = nghieng PHAI, - = nghieng TRAI):")
    t_end = time.time() + 3.0
    while time.time() < t_end:
        if history:
            _, r = history[-1]
            bar = int(r)
            direction = ">>>" if r > 0 else "<<<"
            print(f"  Roll: {r:+6.1f} deg  {direction}", end='\r')
        time.sleep(0.05)
    print()

def analyze():
    print(f"\n[!] Dang phan tich rung lac (4 giay)...")
    print("    -> Nhan nhe vao canh drone ROI BUC TAY RA NGAY!")
    
    # Xoa du lieu cu, doi 4 giay de lay du lieu sach
    history.clear()
    time.sleep(4.0)

    if len(history) < 100:
        print(f"[!] Khong du du lieu ({len(history)} mau). Kiem tra lai ket noi.")
        return

    # Lay du lieu sach
    samples = history.copy()
    n = len(samples)
    
    # Tinh trung binh de bu offset (neu drone dat nghieng)
    avg_r = sum(r for _, r in samples) / n

    # --- Loc nhieu: chi lay bien do lon (> 2 do) moi tinh ---
    NOISE_FLOOR = 2.0  # Do - nguong toi thieu de tinh la rung that

    crossings = 0
    last_sign = 0
    t_first = 0
    t_last = 0
    amplitudes = []
    crossing_times = []

    for t_ms, r in samples:
        rel = r - avg_r
        amplitudes.append(abs(rel))
        if abs(rel) > NOISE_FLOOR:
            sign = 1 if rel > 0 else -1
            if last_sign != 0 and sign != last_sign:
                crossings += 1
                crossing_times.append(t_ms)
                if t_first == 0:
                    t_first = t_ms
                t_last = t_ms
            last_sign = sign

    max_amp = max(amplitudes)
    avg_amp = sum(amplitudes) / len(amplitudes)

    print(f"\n  Bien do toi da: {max_amp:.1f} do | Trung binh: {avg_amp:.1f} do")
    print(f"  So lan chuyen huong: {crossings}")

    if max_amp < NOISE_FLOOR * 1.5:
        print(f"[*] Drone on dinh, khong rung. Hay tang P len (+)")
        return

    if crossings < 6:
        print(f"[*] Chi co {crossings} lan chuyen huong. Can it nhat 6 lan de tinh Tu.")
        print(f"    -> Hay tang P len (+) roi thu lai")
        return

    # Tinh Tu - chu ky rung
    time_span = (t_last - t_first) / 1000.0
    cycles = crossings / 2.0
    tu = time_span / cycles

    # Kiem tra tinh nhat quan cua dao dong
    if tu < 0.1 or tu > 5.0:
        print(f"[!] Chu ky bat thuong: {tu:.3f}s. Co the bi nhieu. Thu lai!")
        return

    # Ziegler-Nichols
    ku = current_p
    kp = 0.6 * ku
    ki = 1.2 * ku / tu
    kd = 0.075 * ku * tu

    print("\n==========================================")
    print(f"🚨 NGƯỠNG RUNG (ULTIMATE GAIN) 🚨")
    print(f" - Ultimate Gain (Ku) : {ku:.2f}")
    print(f" - Chu ky rung  (Tu)  : {tu:.3f} giay ({1.0/tu:.1f} Hz)")
    print(f" - Bien do rung       : {max_amp:.1f} do")
    print("==========================================")
    print(f"🎯 PID TOI UU (Ziegler-Nichols):")
    print(f"   kp_inner[Roll] = {kp:.3f}")
    print(f"   ki_inner[Roll] = {ki:.3f}  (de 0 neu drone on dinh)")
    print(f"   kd_inner[Roll] = {kd:.4f}")
    print("==========================================\n")
    print("[!] BAN CO THE DUNG MOTOR NGAY!")

# Gui P ban dau
time.sleep(1)
ser.write(f"P:{current_p:.2f}\n".encode())

while True:
    try:
        cmd = input(f"\n[Roll P={current_p:.2f}] Lenh (+/-/a/s/q): ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        break

    if cmd == '+':
        current_p = round(current_p + 0.05, 3)
        ser.write(f"P:{current_p:.2f}\n".encode())
        print(f"-> Tang P len {current_p:.2f}")
    elif cmd == '-':
        current_p = round(max(0, current_p - 0.05), 3)
        ser.write(f"P:{current_p:.2f}\n".encode())
        print(f"-> Giam P xuong {current_p:.2f}")
    elif cmd == 'a':
        analyze()
    elif cmd == 's':
        show_realtime()
    elif cmd == 'q':
        ser.close()
        print("Da thoat!")
        break
    else:
        print("Lenh khong hop le. Dung: +, -, a, s, q")
