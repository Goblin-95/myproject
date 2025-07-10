import socket
import time
import numpy as np

HOST = '127.0.0.1'
PORT = 65432

current_pos = np.array([50.0, 50.0])

def animate_to(target_pos, conn):
    global current_pos
    steps = 50
    delay = 0.02  # 20ms per step

    for step in range(steps):
        t = step / (steps - 1)
        new_pos = (1 - t) * current_pos + t * target_pos
        x, y = round(new_pos[0], 2), round(new_pos[1], 2)
        try:
            conn.sendall(f"{x},{y}".encode())
        except Exception as e:
            print("[Sender] Error sending position:", e)
            return
        time.sleep(delay)
    current_pos[:] = target_pos

def start_client():
    print("[Sender] Connecting to GUI...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        while True:
            try:
                s.connect((HOST, PORT))
                print("[Sender] Connected to GUI.")
                break
            except ConnectionRefusedError:
                print("[Sender] Connection refused, retrying in 1s...")
                time.sleep(1)

        while True:
            try:
                data = s.recv(1024)
                if not data:
                    print("[Sender] GUI disconnected.")
                    break
                msg = data.decode().strip()
                if ',' in msg:
                    x_str, y_str = msg.split(',')
                    x, y = float(x_str), float(y_str)
                    print(f"[Sender] Received target: ({x}, {y})")
                    animate_to(np.array([x, y]), s)
            except Exception as e:
                print("[Sender] Error:", e)
                break

if __name__ == "__main__":
    start_client()
