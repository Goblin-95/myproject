import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 2 * np.pi, 100)
y = np.sin(x)

plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
line, = ax.plot(x, y)

for i in range(100):
    y = np.sin(x + i * 0.1)     # Shift the sine wave
    line.set_ydata(y)           # Update the line with new data
    plt.pause(0.05)             # Pause briefly and refresh the plot

plt.ioff()  # Optional: turn off interactive mode
plt.show()  # Final display
