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

bg_selected = False
char_selected = False
presets_applied = False

baseDir = os.path.dirname(os.path.abspath(__file__))
background_path = os.path.join(baseDir, "backgrounds", "grass backgroundusable.jpg")
object_path = os.path.join(baseDir, "objects", "dino.png")

window = tk.Tk()
window.title("Live Animation Viewer")



def resetObject():
    updatePosition(50, 50)
    if conn is not None:
        try:
            speed = speedVar.get()
            conn.sendall(f"50,50,{speed}".encode()) # notify sender
            print("[GUI] Sent reset to center (50,50)")
        except Exception as e:
            print("[GUI] Error sending reset:", e)

def updateObjectSize():
    global imgSize
    if not presets_applied or imageArtist is None:
        return
    imgSize = sizeVar.get()
    updatePosition(currentPos[0], currentPos[1])


fig, ax = plt.subplots(figsize=(9, 9))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_title("Click to set target point")

# Create a main frame to hold canvas and controls side by side
mainFrame = tk.Frame(window)
mainFrame.pack(fill=tk.BOTH, expand=True)

# Pack canvas on the left inside mainFrame
canvas = FigureCanvasTkAgg(fig, master=mainFrame)
canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

# Create control frame on the right side inside mainFrame
controlFrame = tk.Frame(mainFrame)
controlFrame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)

bgVar = tk.StringVar(value="Select Background")
charVar = tk.StringVar(value="Select Character")
sizeVar = tk.IntVar(value=imgSize)  # initial size = 10
speedVar = tk.StringVar(value="normal")

tk.Label(controlFrame, text="Animation Speed:").pack(anchor="w")
tk.OptionMenu(controlFrame, speedVar, "slow", "normal", "fast").pack(fill=tk.X)

bg_options = ["Grass", "Sky"]
char_options = ["Person", "Dino"]

tk.Label(controlFrame, text="Choose Background:").pack(anchor="w")
bg_menu = tk.OptionMenu(controlFrame, bgVar, *bg_options)
bg_menu.pack(fill=tk.X)

tk.Label(controlFrame, text="Choose Character:").pack(anchor="w")
char_menu = tk.OptionMenu(controlFrame, charVar, *char_options)
char_menu.pack(fill=tk.X)


def applyPresets():
    global background_path, object_path, presets_applied
    bg = bgVar.get()
    char = charVar.get()

    if bg == "Grass":
        background_path = os.path.join(baseDir, "backgrounds", "grass backgroundusable.jpg")
    if bg == "Sky":
        background_path = os.path.join(baseDir, "backgrounds", "sky background.png")
    # add more backgrounds if needed

    if char == "Person":
        object_path = os.path.join(baseDir, "objects", "person.webp")
    if char == "Dino":
        object_path = os.path.join(baseDir, "objects", "dino.png")
        
    # add more characters if needed

    presets_applied = True
    loadScene()  # Only now load the scene

apply_button = tk.Button(controlFrame, text="Apply Presets", command=applyPresets)
apply_button.pack(fill=tk.X, pady=10)

tk.Label(controlFrame, text="Character Size:").pack(anchor="w")
tk.Scale(controlFrame, from_=5, to=30, orient=tk.HORIZONTAL,
         variable=sizeVar, command=lambda val: updateObjectSize()).pack(fill=tk.X, pady=5)

reset_button = tk.Button(controlFrame, text="Center Character", command=resetObject)
reset_button.pack(fill=tk.X, pady=10)



def loadScene():
    global imageArtist

    if not presets_applied:
        return 
    
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
    if not presets_applied or imageArtist is None:
        return
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
                parts = msg.split(',')
                if len(parts) >= 2:
                    try:
                        x = float(parts[0].strip())
                        y = float(parts[1].strip())
                        updatePosition(x, y)
                    except ValueError:
                        print("[GUI] Received invalid position:", msg)
                else:
                    print("[GUI] Received malformed message:", msg)
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
    speed = speedVar.get()
    try:
        conn.sendall(f"{x},{y},{speed}".encode())
        print(f"[GUI] Sent target: ({x}, {y}) with speed {speed}")
    except Exception as e:
        print("[GUI] Error sending target:", e)



threading.Thread(target=socket_server, daemon=True).start()
fig.canvas.mpl_connect('button_press_event', onClick)


window.mainloop()