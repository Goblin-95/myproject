# === Imports ===
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.image as mpimg
from PIL import Image
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

canvas = FigureCanvasTkAgg(fig, master=window)
canvas.get_tk_widget().pack()


# === Control Panel ===
controlFrame = tk.Frame(window)
controlFrame.pack()

# Background selector
bgVar = tk.StringVar(value="Grass")
tk.Label(controlFrame, text="Background:").pack(side=tk.LEFT, padx=5)
ttk.Combobox(controlFrame, textvariable=bgVar, values=list(backgrounds.keys()), state="readonly").pack(side=tk.LEFT)

# Object selector
objVar = tk.StringVar(value="Person")
tk.Label(controlFrame, text="Object:").pack(side=tk.LEFT, padx=5)
ttk.Combobox(controlFrame, textvariable=objVar, values=list(objects.keys()), state="readonly").pack(side=tk.LEFT)

# Size slider
sizeVar = tk.IntVar(value=imgSize)
sizeSlider = tk.Scale(controlFrame, from_=5, to=30, label="Object Size",
                      variable=sizeVar, orient=tk.HORIZONTAL, command=lambda e: updateObjectSize())
sizeSlider.pack(side=tk.LEFT, padx=10)

# Speed selector
speedVar = tk.StringVar(value="Medium")
tk.Label(controlFrame, text="Speed:").pack(side=tk.LEFT, padx=5)
ttk.Combobox(controlFrame, textvariable=speedVar, values=["Slow", "Medium", "Fast"], state="readonly").pack(side=tk.LEFT, padx=5)


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

    from matplotlib import image as mpimg_module
    from PIL import Image

    # Load object
    img = mpimg.imread(objects[objVar.get()])
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



# Buttons
tk.Button(controlFrame, text="Load Presets", command=loadPresets).pack(side=tk.LEFT, padx=10)
tk.Button(controlFrame, text="Reset Position", command=resetObject).pack(side=tk.LEFT, padx=10)


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


# === Event Binding & Main Loop ===
fig.canvas.mpl_connect('button_press_event', onClick)
window.mainloop()