# === Imports ===
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.image as mpimg
from PIL import Image
import threading
import socket
import os

# === Globals & Initial Setup ===
animating = False
targetQueue = []
imgSize = 10

start = np.array([50, 50])
target = np.array([50, 50])

baseDir = os.path.dirname(os.path.abspath(__file__))
backgrounds = {
    "Grass": os.path.join(baseDir, "backgrounds", "grass backgroundusable.jpg"),
    "Sky": os.path.join(baseDir, "backgrounds", "sky background.png")
}
objects = {
    "Person": os.path.join(baseDir, "objects", "person.webp"),
    "Dinosaur": os.path.join(baseDir, "objects", "dino.png")
}

# === Main Window Setup ===
window = tk.Tk()
window.title("Point & Click")

fig, ax = plt.subplots(figsize=(9, 9))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_title("Click to set target point")

# === Frame Layout ===
mainFrame = tk.Frame(window)
mainFrame.pack(fill=tk.BOTH, expand=True)

# Plot canvas on the left
canvas = FigureCanvasTkAgg(fig, master=mainFrame)
canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

# Control panel on the right
controlFrame = tk.Frame(mainFrame)
controlFrame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)

# === Control Variables ===
bgVar = tk.StringVar(value="Grass")        # Stores selected background
objVar = tk.StringVar(value="Person")      # Stores selected character
sizeVar = tk.IntVar(value=imgSize)         # Stores object size from slider
speedVar = tk.StringVar(value="Medium")    # Stores selected speed

# === GUI Controls ===

# --- Background selector dropdown ---
tk.Label(controlFrame, text="Choose Background:").pack(anchor="w", pady=5)
ttk.Combobox(controlFrame, textvariable=bgVar, values=list(backgrounds.keys()), state="readonly").pack(fill=tk.X)

# --- Object (character) selector dropdown ---
tk.Label(controlFrame, text="Choose Character:").pack(anchor="w", pady=5)
ttk.Combobox(controlFrame, textvariable=objVar, values=list(objects.keys()), state="readonly").pack(fill=tk.X)

# --- Load selected background and character ---
tk.Button(controlFrame, text="Apply Background & Character", command=lambda: loadPresets()).pack(fill=tk.X, pady=10)

# --- Reset character to center position ---
tk.Button(controlFrame, text="Center Character", command=lambda: resetObject()).pack(fill=tk.X, pady=5)

# --- Object size slider ---
tk.Label(controlFrame, text="Character Size:").pack(anchor="w", pady=5)
sizeSlider = tk.Scale(controlFrame, from_=5, to=30, variable=sizeVar,
                      orient=tk.HORIZONTAL, command=lambda e: updateObjectSize())
sizeSlider.pack(fill=tk.X, pady=5)

# --- Movement speed selector dropdown ---
tk.Label(controlFrame, text="Movement Speed:").pack(anchor="w", pady=5)
ttk.Combobox(controlFrame, textvariable=speedVar, values=["Slow", "Medium", "Fast"], state="readonly").pack(fill=tk.X)

# === GUI Functions ===

def resetObject():
    """Reset object back to center"""
    global start
    start[:] = [50, 50]
    size = sizeVar.get()
    imageArtist.set_extent([
        start[0], start[0] + size,
        start[1], start[1] + size
    ])
    canvas.draw()

def updateObjectSize():
    """Update object size live when slider moves"""
    size = sizeVar.get()
    imageArtist.set_extent([
        start[0], start[0] + size,
        start[1], start[1] + size
    ])
    canvas.draw_idle()

def loadPresets():
    """Set background and object image based on current dropdowns"""
    global imageArtist, img, start

    ax.clear()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_title("Click to set target point")

    # Load background
    bgImg = mpimg.imread(backgrounds[bgVar.get()])
    ax.imshow(bgImg, extent=[0, 100, 0, 100])

    # Load object image
    imgPath = objects[objVar.get()]
    pilImg = Image.open(imgPath).resize((128, 128))
    img = np.array(pilImg)

    start[:] = [50, 50]
    size = sizeVar.get()
    imageArtist = ax.imshow(img, extent=[
        start[0], start[0] + size,
        start[1], start[1] + size
    ])

    canvas.draw()

# === Animation Logic ===

def onClick(event):
    """Queue target on mouse click"""
    if event.xdata is None or event.ydata is None:
        return
    targetQueue.append(np.array([event.xdata, event.ydata]))
    startNext()

def startNext():
    """Start animation to the next target"""
    global anim, animating, dynamicSteps

    if animating or not targetQueue:
        return

    nextTarget = targetQueue.pop(0)
    distance = np.linalg.norm(nextTarget - start)

    # Set total animation time depending on selected speed
    speed = speedVar.get()
    if speed == "Slow":
        total_duration = 5000
    elif speed == "Fast":
        total_duration = 500
    else:
        total_duration = 1500

    interval = 20  # ms between frames
    dynamicSteps = max(int(total_duration / interval), 10)

    animating = True
    anim = FuncAnimation(fig, update, frames=dynamicSteps,
                         fargs=(start.copy(), nextTarget.copy()),
                         interval=interval, repeat=False)
    canvas.draw()

def update(frame, startPos, nextTarget):
    """Move object smoothly toward the target"""
    global start, animating, dynamicSteps

    t = frame / (dynamicSteps - 1)
    newPos = (1 - t) * startPos + t * nextTarget

    size = sizeVar.get()
    imageArtist.set_extent([
        newPos[0], newPos[0] + size,
        newPos[1], newPos[1] + size
    ])
    canvas.draw_idle()

    if frame == dynamicSteps - 1:
        start[:] = nextTarget[:]
        animating = False
        startNext()


def socket_listener():
    HOST = '127.0.0.1'
    PORT = 65432

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print("Waiting for connection from sender...")

        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    x_str, y_str = data.decode().strip().split(',')
                    x, y = float(x_str), float(y_str)
                    print(f"Received point: ({x}, {y})")
                    targetQueue.append(np.array([x, y]))
                    startNext()
                except ValueError:
                    print("Invalid data received:", data)


# === Event Binding & Main Loop ===
listener_thread = threading.Thread(target=socket_listener, daemon=True)
listener_thread.start()
fig.canvas.mpl_connect('button_press_event', onClick)
window.mainloop()
