import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from tkinter import filedialog
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.image as mpimg
import time

window = tk.Tk()
window.title("Point & Click")

fig, ax = plt.subplots(figsize=(9, 9))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_title("Click to set target point")

start = np.array([50, 50])
target = np.array([50, 50])

img_path = filedialog.askopenfilename(title="Select Image", filetypes=[("PNG Images", "*.png"), ("All Files", "*.*")])
if not img_path:
    raise Exception("No image selected")

img = mpimg.imread(img_path)
img_size = 10 

image_artist = ax.imshow(img, extent=[start[0], start[0] + img_size, start[1], start[1] + img_size])

steps = 100
anim = None

canvas = FigureCanvasTkAgg(fig, master=window)
canvas.get_tk_widget().pack()

def onClick(event):
    global target
    global currentStep
    global anim

    target[:] = [event.xdata, event.ydata]

    if anim:
        anim.event_source.stop()
    anim = FuncAnimation(fig, update, frames=steps, interval=20, repeat=False)
    canvas.draw()

def update(frame):
    global start, target

    t = frame / (steps - 1)
    new_pos = (1 - t) * start + t * target

    image_artist.set_extent([new_pos[0], new_pos[0] + img_size, new_pos[1], new_pos[1] + img_size])
    canvas.draw_idle()

    if frame == steps - 1:
        start[:] = target[:]


fig.canvas.mpl_connect('button_press_event', onClick)

window.mainloop()