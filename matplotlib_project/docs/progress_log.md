# Progress Log

## June 30, 2025

- Installed matplotlib using `pip install matplotlib`
- Created initial project folder structure
- Wrote first basic line plot using `pyplot`
  - Code saved in `basics/basic_plots.py`
- Successfully ran `plt.show` and saw the output window

- Created a sine wave plot
  - Code saved in `shapes/sine_circle_random.py`

- Looked into animating the sine wave
- Learned how `plt.pause()` works to refresh the plot during animations
- Understood that animation is created by:
  - Updating the data (`y` values)
  - Applying the update to the line object
  - Refreshing the plot with `plt.pause(...)` to display the new frame


- Created a basic Tkinter window with a button that triggers a plot
- Learned how to embed a Matplotlib figure inside a Tkinter GUI using `FigureCanvasTkAgg`
- Understood that the key steps are:
  - Creating a Matplotlib figure and axes with `plt.subplots()`
  - Plotting data on the axes (`ax.plot(...)`)
  - Wrapping the figure in a `FigureCanvasTkAgg` object linked to the Tkinter window
  - Calling `.draw()` on the canvas to render the plot
  - Adding the canvas widget to the Tkinter window layout with `.get_tk_widget().pack()`
- Recognized that `plt.show()` should NOT be used in a Tkinter GUI as it blocks the window
- Successfully connected a button’s `command` to the plot function, so the plot appears when clicked


