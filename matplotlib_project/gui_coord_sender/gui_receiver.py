import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import threading
import socket
import os

currentPos = np.array([50, 50])
imgSize = 10
imageArtist = None
conn = None  # socket connection with sender

baseDir = os.path.dirname(os.path.abspath(__file__))
background_path = os.path.join(baseDir, "backgrounds", "grass backgroundusable.jpg")
object_path = os.path.join(baseDir, "objects", "dino.png")

window = tk.Tk()
window.title("Live Animation Viewer")

fig, ax = plt.subplots(figsize=(9, 9))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_title("Click to set target point")
canvas = FigureCanvasTkAgg(fig, master=window)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

def loadScene():
    global imageArtist
    ax.clear()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_title("Click to set target point")

    bgImg = plt.imread(background_path)
    ax.imshow(bgImg, extent=[0, 100, 0, 100])

    objImg = Image.open(object_path).resize((128, 128))
    img = np.array(objImg)

    imageArtist = ax.imshow(img, extent=[
        currentPos[0], currentPos[0] + imgSize,
        currentPos[1], currentPos[1] + imgSize
    ])
    canvas.draw()

def updatePosition(x, y):
    global currentPos
    currentPos[:] = [x, y]
    imageArtist.set_extent([
        x, x + imgSize,
        y, y + imgSize
    ])
    canvas.draw_idle()

def handle_sender(conn_socket):
    global conn
    conn = conn_socket
    print("[GUI] Sender connected.")

    def recv_loop():
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    print("[GUI] Sender disconnected.")
                    break
                msg = data.decode().strip()
                if ',' in msg:
                    x_str, y_str = msg.split(',')
                    x, y = float(x_str), float(y_str)
                    # print(f"[GUI] Received position update: ({x}, {y})")
                    updatePosition(x, y)
            except Exception as e:
                print("[GUI] Error receiving data:", e)
                break

    threading.Thread(target=recv_loop, daemon=True).start()

def socket_server():
    HOST = '127.0.0.1'
    PORT = 65432
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[GUI] Waiting for sender to connect on {HOST}:{PORT} ...")
        conn_socket, addr = s.accept()
        handle_sender(conn_socket)

def onClick(event):
    if event.xdata is None or event.ydata is None or conn is None:
        return
    x, y = round(event.xdata, 2), round(event.ydata, 2)
    try:
        conn.sendall(f"{x},{y}".encode())
        print(f"[GUI] Sent target: ({x}, {y})")
    except Exception as e:
        print("[GUI] Error sending target:", e)

loadScene()
threading.Thread(target=socket_server, daemon=True).start()
fig.canvas.mpl_connect('button_press_event', onClick)
window.mainloop()
