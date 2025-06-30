import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

def plot():
    fig, ax = plt.subplots()
    ax.plot([1, 2, 3], [1, 4, 9]) 

    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    canvas.get_tk_widget().pack()

window = tk.Tk()
window.title("Matplotlib GUI")
button = tk.Button(window, text="Plot", command=plot)
button.pack()
window.mainloop()