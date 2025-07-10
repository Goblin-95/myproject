import socket
import time


HOST = '127.0.0.1'  # localhost
PORT = 65432        # must match receiver
# python "C:\Users\georg\python projects all\matplotlibPROJECT\matplotlib_project\gui_coord\sender.py"
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    # Send example target points over time
    for point in [(10, 10), (30, 30), (70, 70)]:
        message = f"{point[0]},{point[1]}"
        print(f"Sending: {message}")
        s.sendall(message.encode())
        time.sleep(2) 