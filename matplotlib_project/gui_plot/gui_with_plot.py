import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from tkinter import filedialog
from matplotlib.animation import FuncAnimation, PillowWriter
import time

lineColor = 'blue' # global variable to save color
lineStyle = '-'
gridOn = False
buttonsPacked = False


def setColorRed():
    global lineColor
    lineColor = 'red'
    selectStyle()

def setColorGreen():
    global lineColor
    lineColor = 'green'
    selectStyle()

def setColorBlue():
    global lineColor
    lineColor = 'blue'
    selectStyle()

def selectColor():
    buttonPlot.pack_forget()
    buttonRed.pack()
    buttonGreen.pack()
    buttonBlue.pack()


#Styling

def selectStyle():
    
    buttonRed.pack_forget()
    buttonGreen.pack_forget()
    buttonBlue.pack_forget()
    
    buttonSolid.pack()
    buttonDashed.pack()
    buttonDotted.pack()

    colorFrame.pack_forget()

def setStyle(style):
    global lineStyle
    lineStyle = style

    buttonSolid.pack_forget()
    buttonDashed.pack_forget()
    buttonDotted.pack_forget()
    plot()

def toggleGrid():
    global gridOn
    gridOn = not gridOn
    plot()


def plot():

    global fig
    global buttonsPacked
   
    for widget in plotFrame.winfo_children():
        widget.destroy()
    
    styleFrame.pack_forget()
    
    fig, ax = plt.subplots()
    x = np.linspace(0, 2 * np.pi, 100)
    y = np.sin(x)
    line, = ax.plot(x, y, color = lineColor, linestyle = lineStyle)

    canvas = FigureCanvasTkAgg(fig, master=plotFrame)
    canvas.draw()
    canvas.get_tk_widget().pack()


    if not buttonsPacked:
        buttonSaveImage.pack()
        buttonSaveAnimation.pack()
        buttonToggleGrid.pack()
        
        buttonsPacked = True

    ax.grid(gridOn)


    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")

    for i in range(100):
        y = np.sin(x + i * 0.1)     # Shift the sine wave
        line.set_ydata(y)           # Update the line with new data
        canvas.draw()
        window.update()
        time.sleep(0.05)



def saveImage():

    file_path = filedialog.asksaveasfilename(defaultextension=".png",
    filetypes=[("PNG files", "*.png"),
    ("PDF files", "*.pdf"),
    ("All files", "*.*")])

    if file_path:
        fig.savefig(file_path)

def saveGIF():

    global fig

    fig, ax = plt.subplots()
    x = np.linspace(0, 2 * np.pi, 100)
    line, = ax.plot(x, np.sin(x), color=lineColor, linestyle=lineStyle)

    def update(i):
        y = np.sin(x + i * 0.1)
        line.set_ydata(y)
        return line,

    ani = FuncAnimation(fig, update, frames=100, interval=50, blit=True)

    file_path = filedialog.asksaveasfilename(defaultextension=".gif",
                                             filetypes=[("GIF files", "*.gif")])
    if file_path:
        ani.save(file_path, writer=PillowWriter(fps=20))




window = tk.Tk()
window.title("Plot")

controlFrame = tk.Frame(window)
colorFrame = tk.LabelFrame(window, text = "Color")
styleFrame = tk.LabelFrame(window, text="Style")
controlFrame.pack()
colorFrame.pack()
styleFrame.pack()

buttonPlot = tk.Button(window, text="Plot", command=selectColor)
buttonPlot.pack()

#Color Buttons
buttonRed = tk.Button(colorFrame, text="Red", command=setColorRed)
buttonGreen = tk.Button(colorFrame, text="Green", command=setColorGreen)
buttonBlue = tk.Button(colorFrame, text="Blue", command=setColorBlue)



#Style Buttons

buttonSolid = tk.Button(styleFrame, text="Solid (-)", command=lambda: setStyle('-'))
buttonDashed = tk.Button(styleFrame, text="Dashed (--)", command=lambda: setStyle('--'))
buttonDotted = tk.Button(styleFrame, text="Dotted (:)", command=lambda: setStyle(':'))

buttonToggleGrid = tk.Button(controlFrame, text="Toggle Grid", command=toggleGrid)
buttonSaveImage = tk.Button(controlFrame, text="Save Image", command=saveImage)
buttonSaveAnimation = tk.Button(controlFrame, text="Save Animation", command=saveGIF)

#def saveImage():





plotFrame = tk.Frame(window)
plotFrame.pack()

window.mainloop()