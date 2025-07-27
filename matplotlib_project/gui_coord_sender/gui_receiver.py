# ===================== Imports ===================== #
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from staticmap import StaticMap, CircleMarker
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import threading
import socket
import os

# ===================== Globals & Initial Config ===================== #
currentPos = np.array([50, 50])
imgSize = 10
imageArtist = None
conn = None  # socket connection with sender

presetsApplied = False

baseDir = os.path.dirname(os.path.abspath(__file__))
backgroundPath = os.path.join(baseDir, "backgrounds", "grass backgroundusable.jpg")
objectPath = os.path.join(baseDir, "objects", "dino.png")

# ===================== GUI Window Setup ===================== #
window = tk.Tk()
window.title("Live Animation Viewer")

# ===================== Utility Functions ===================== #
def resetObject():
    updatePosition(50, 50)
    if conn is not None:
        try:
            speed = speedVar.get()
            conn.sendall(f"50,50,{speed}".encode())
            print("[GUI] Sent reset to center (50,50)")
        except Exception as e:
            print("[GUI] Error sending reset:", e)

def updateObjectSize():
    global imgSize
    if not presetsApplied or imageArtist is None:
        return
    imgSize = sizeVar.get()
    updatePosition(currentPos[0], currentPos[1])

# ===================== Plot Area Setup ===================== #
fig, ax = plt.subplots(figsize=(9, 9))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_title("Click to set target point")

# ===================== GUI Layout Structure ===================== #
mainFrame = tk.Frame(window)
mainFrame.pack(fill=tk.BOTH, expand=True)

canvas = FigureCanvasTkAgg(fig, master=mainFrame)
canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

controlFrame = tk.Frame(mainFrame)
controlFrame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)

# ===================== Control Variables ===================== #
bgOptions = ["Grass", "Sky", "GPS Map"]
charOptions = ["Person", "Dino"]

bgVar = tk.StringVar(value="Select Background")
charVar = tk.StringVar(value="Select Character")
sizeVar = tk.IntVar(value=imgSize)
speedVar = tk.StringVar(value="normal")
zoomVar = tk.IntVar(value=15)

# ===================== Control Widgets ===================== #
tk.Label(controlFrame, text="Animation Speed:").pack(anchor="w")
tk.OptionMenu(controlFrame, speedVar, "slow", "normal", "fast").pack(fill=tk.X)

tk.Label(controlFrame, text="Choose Background:").pack(anchor="w")
tk.OptionMenu(controlFrame, bgVar, *bgOptions).pack(fill=tk.X)

tk.Label(controlFrame, text="Choose Character:").pack(anchor="w")
tk.OptionMenu(controlFrame, charVar, *charOptions).pack(fill=tk.X)

tk.Label(controlFrame, text="GPS Coordinates (lat,lon):").pack(anchor="w")
gpsEntry = tk.Entry(controlFrame)
gpsEntry.pack(fill=tk.X)

gpsEntry.insert(0, "41.8755, -87.6497") 

tk.Label(controlFrame, text="📍 TurtleBot GPS Mode", font=("Arial", 8), fg="blue").pack(anchor="w")
tk.Label(controlFrame, text="Use coordinates above", font=("Arial", 8), fg="gray").pack(anchor="w")

print("[DEBUG] GPS entry field created")

tk.Label(controlFrame, text="GPS Zoom Level:").pack(anchor="w")
tk.Scale(controlFrame, from_=10, to=20, orient=tk.HORIZONTAL, variable=zoomVar).pack(fill=tk.X, pady=5)

# ===================== Preset Application ===================== #
def applyPresets():
    global backgroundPath, objectPath, presetsApplied

    bg = bgVar.get()
    char = charVar.get()

    # Background selection
    if bg == "Grass":
        backgroundPath = os.path.join(baseDir, "backgrounds", "grass backgroundusable.jpg")
    elif bg == "Sky":
        backgroundPath = os.path.join(baseDir, "backgrounds", "sky background.png")
    elif bg == "GPS Map":
        gpsText = gpsEntry.get()
        try:
            latStr, lonStr = gpsText.split(',')
            lat, lon = float(latStr.strip()), float(lonStr.strip())
        except Exception:
            print("[GUI] Invalid GPS input. Defaulting to Chicago, Illinois.")
            lat, lon = 41.8781, -87.6298

        zoomLevel = zoomVar.get()
        bgImgArray = getMapAsNpArray(lat, lon, zoom=zoomLevel)
        tempPath = os.path.join(baseDir, "temp_map.png")
        plt.imsave(tempPath, bgImgArray)
        backgroundPath = tempPath

    # Character selection
    if char == "Person":
        objectPath = os.path.join(baseDir, "objects", "person.webp")
    elif char == "Dino":
        objectPath = os.path.join(baseDir, "objects", "dino.png")

    presetsApplied = True
    loadScene()

# ===================== Preset Buttons ===================== #
tk.Button(controlFrame, text="Apply Presets", command=applyPresets).pack(fill=tk.X, pady=10)

tk.Label(controlFrame, text="Character Size:").pack(anchor="w")
tk.Scale(controlFrame, from_=5, to=30, orient=tk.HORIZONTAL, variable=sizeVar,
         command=lambda val: updateObjectSize()).pack(fill=tk.X, pady=5)

tk.Button(controlFrame, text="Center Character", command=resetObject).pack(fill=tk.X, pady=10)

# ===================== Scene Loading ===================== #
def loadScene():
    global imageArtist

    if not presetsApplied:
        return

    ax.clear()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_title("Click to set target point")

    bgImg = plt.imread(backgroundPath)
    ax.imshow(bgImg, extent=[0, 100, 0, 100])

    objImg = Image.open(objectPath).resize((128, 128))
    img = np.array(objImg)

    imageArtist = ax.imshow(img, extent=[
        currentPos[0], currentPos[0] + imgSize,
        currentPos[1], currentPos[1] + imgSize
    ])
    canvas.draw()

# ===================== Map Generation Helpers ===================== #
def generateMapImage(lat, lon, zoom=14, size=(600, 600)):
    m = StaticMap(size[0], size[1])
    marker = CircleMarker((lon, lat), 'red', 12)
    m.add_marker(marker)
    image = m.render(zoom=zoom)
    return image

def getMapAsNpArray(lat, lon, zoom=14):
    img = generateMapImage(lat, lon, zoom=zoom)
    return np.array(img)

# ===================== Position Update ===================== #
def updatePosition(x, y):
    global currentPos
    if not presetsApplied or imageArtist is None:
        return
    currentPos[:] = [x, y]
    imageArtist.set_extent([
        x, x + imgSize,
        y, y + imgSize
    ])
    canvas.draw_idle()

# ===================== Socket Handling ===================== #
def handleSender(connSocket):
    global conn
    conn = connSocket
    print("[GUI] Sender connected.")

    def recvLoop():
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

    threading.Thread(target=recvLoop, daemon=True).start()

def socketServer():
    HOST = '127.0.0.1'
    PORT = 65432
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[GUI] Waiting for sender to connect on {HOST}:{PORT} ...")
        connSocket, addr = s.accept()
        handleSender(connSocket)

# ===================== Mouse Click Event ===================== #
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

# ===================== App Initialization ===================== #
threading.Thread(target=socketServer, daemon=True).start()
fig.canvas.mpl_connect('button_press_event', onClick)
window.mainloop()
