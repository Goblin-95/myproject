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

## July 1st, 2025

- Called Safwan on Zoom and updated him on my progress
- Was assigned new tasks:
  - Add more features to your existing GUI. For example, add buttons to change line color, style, add legends, grid, labels, etc.- Allow saving plots as images (PNG/PDF) and animations as video files (like .mp4).
  - Be creative in how the plot, animation, and GUI look and behave.
  - Create an animation where an object moves from a start point to a target point.
- Spent time researching how to approach the problem
- Spent time watching YouTube videos on Python basics to build a better foundation in the language
- Related concepts from C to Python like global variables, loops, and objects



## July 2nd, 2025

- Goal for today is to apply the animation to the existing GUI with plot

- Started to add color options to the plot
- Succesfully added buttons that will change color of the line in the plot, but buttons look janky and incorrectly placed
  will try to make this look better
- Swapped the pack order so the button to chnage colors shows up only after you have clicked plot
- Added a title to the frame that holds the colors and removed the plot button after you have clicked it
- Added blue color to return to default color once you change it

- Started to add the animation to the GUI
- Looked into how the animation code would change from the `simple_animation` code when applied to this GUI
- Understood differences between canvas.draw() and plt.pause(). One is for Tkinter and the other is for MatPlotLib
- Understood that the code works by updating the y values, redrawing the line with canvas.draw, updating it with window.update to actually show on the GUI
  this is the basic logic behind GUI animations
- Animation works fine now

- The GUI first plays the animation and only then do you select the color, I want to make it select the color first before playing it
- Thought about how to do this
- Added new function `selectColor` that runs after you click plot, and only after you have chosen a color does the plot actually animate

- Started to add style options to the plot
- Realized that the style should be selected after the color, just like how color is selected after pressing "Plot"
- Created a new function selectStyle that is triggered after choosing a color
- Made sure that the color buttons disappear when selecting a style to keep the interface clean
- Added three style options: solid, dashed, and dotted — each represented by its respective button
- Each style button calls a function setStyle(style) which sets the global lineStyle and then runs the animation
- Updated the plot function to use both lineColor and lineStyle when drawing the sine wave
- Now after clicking "Plot" the user chooses a color then a style, and only then does the animation run with both properties correctly applied
- Confirmed that the line animates using the selected color and line style as expected

- Buttons dissapear after you click them but the frame remains, want to remove that.
- Added pack.forget() operations in selectStyle and setStyle to remove the buttons and frame once you are done with them

- Spent more time researching and watching videos about python
- Related more concepts from python to C
- Looked into how structs translate from C to Python and differences in their usage and concepts



July 3rd, 2025:

- Goal for today is to finish up the tasks in the email

- Started to look into how to allow for saving a pdf of the graph
- Added a Save Image button that only shows up after the user has created a graph
- Imported the library that allows for saving
- Looked into how the saving file function actually works
- Successfully created a PDF save option and saved a pdf of the graph
- Imported matplotlib library that allows for mp4 saving
- Looked into what the library actually does
- Saving as an MP4 requires downloading things and a ton of setup it seems, so I will opt instead to download as a .gif which seems to be the easier option
- Looked into how to save as gif instead
- Successfully created an option to save as .gif and saved a .gif of the graph

- Now will try to add a toggleable grid and labels to the graph
- Added labels, very simple
- Looked into adding a grid 
- Want to start with grid off, and then have the option to toggle it as the aniamtion runs
- Problem arises as the way my code is now, everytime I click toggle grid, it repacks the buttons udner the previous ones
- Will now track the packing of the buttons with a boolean so they only pack once
- Succesfully added a toggleable grid

- Now will begin to work on Coordinate-Based Animation
- Created new file to work on, done with the plot
- Looked into how to do this
- Created a canvas to start with
- Adjusted size to be pretty large on the screen
- Want to start with just a dot moving around, but later would like to figure out how to replace it with an image
- Created red dot object starting at the center
- This animation will use FuncAnimation instead of manual animation like before
- Successfully made a red dot that starts in the center go to where you click on the canvas. 
- Took time to understand the code more

- Looked into how to change the object moving to an uploadable image
- The GUI now opens the file explorer so that you choose a image to act as the object
- Object starts at center and goes to where the user clicks



July 4th, 2025:

- Todays goal is to further develop the point and click GUI and make it more creative

- Right now, the object only goes from the center to where you click one time but I would like it to continue to move as you keep clicking
- Spent time looking how to do this
- Thought up of how to logically lay this problem out
- Considering adding a queue in which as you click a queue of targets that the object will go to will appear
- Looked into how to do this
- Will use an array named targetQueue
- Need to store the clicks and add them to array
- Made a list called targetQueue to store each place the user clicks
- Changed the onClick() function to add each click to that list and try to start movement if nothing is moving
- Made a startNext() function that checks if the object is already moving, and if not, starts moving to the next spot in the list
- Made an update() function that slowly moves the image from where it is to the next spot using animation
- Noticed the object only moved once, even after clicking more times
- Found out the issue was that the program wasn’t correctly marking the animation as finished
- Fixed that by adding global animating so the flag updates properly
- After the fix, tested and confirmed the object now moves through every clicked spot one at a time

- Now I want to add the ability to add a background image to the GUI
- I want it to prompt the user to include a background image before the object
- Looked into how to do this
- Added a prompt that asks for a background image
- Wraps the image around the canvas



July 5th, 2025:

- Goal for today is to complete the rest of the tasks in the email
- Wanted to brainstorm more things I could add to this GUI

- The whole choosing images thing is not very graceful and basically requires the user to have images downlaoded before they use the GUI
- Would like to create preset options for object and background that the user could choose from to interact with
- Looked into how to do this
- Created a folder that has the image preset options
- Found images to act as presets from creative commons sources 
- Added dropdown menus so the user can pick from preset options
- Added a “Load Presets” button to apply the selected background and object
- Fixed an issue where some of the code was running before the window was fully set up
- Renamed all the variables to use the same consistent style
- Fixed a bug where the images couldn’t be found by making sure the paths were correct no matter where the program is run from
- After making all these changes, the program now works better and is easier for users to interact with without needing to upload their own images manually

- Noticed how slow the object moves if you click a target that is relatively close to it
- Looked into this issue
- Found out issue to be that the animation will always take 100 steps, meaning that if its close it will run 100 frames of it when that is not neccessary
- Would be better to add a step value that is not set but instead works dynamically with the distance betweent he current positon and its next target
- Fixed a bug that forgot to calculate the distance
- Successfully fixed the slow movement issue
- But now, the animation looks very choppy, want to look if I can make it smoother
- Realized the animation was choppy because it was using the live position (start) which was changing during the animation.
- Fixed this by giving the animation a frozen copy of the starting point so it always moves smoothly from where it began.
- Updated the animation function to use that frozen start and only update the real position after the move is finished.
- This made the movement look smooth the whole way, even on short distances.

- I want to add a reset button to the GUI so the object is reset after it is moved so you dont have to reopen the GUI each time
- Added a resetObject function
- Added a "Reset Position" button

- The resetObject function sets the object’s position back to the center and updates its location on the canvas
- The button calls this function when clicked so you can reset the object anytime without restarting the program
- Also made sure the canvas redraws right away so the change is visible instantly



July 7th, 2025:

- Goal for today is to further develop the GUI with more features and whatnot

- I want to add a slider that changes the size of the object live
- Looked into how to do this
- Added a size slider widget linked to an IntVar to control object size
- Created an updateObjectSize function that resizes the object image when the slider moves
- Integrated the slider’s value into loadPresets, resetObject, and the animation update function so size changes apply everywhere
- Fixed a bug by properly initializing the imageArtist inside loadPresets to display the object image correctly
- Ensured the canvas redraws immediately after resizing or resetting for instant visual feedback

- Now I want to add speed options
- Looked into how to do this
- Wanted to create three options, slow, medium, and fast
- Added a StringVar and a ttk.Combobox to the GUI to let the user choose the speed
- At first I tried changing the animation interval directly based on the speed setting, but the object still looked like it was moving the same speed no matter what
- Realized that the interval alone wasn’t enough also needed to change how many steps the animation plays to actually change the overall duration
- Decided to keep the frame interval fixed for smoother animation, and instead change the total duration by adjusting the number of frames
- Set slow to be 3 seconds, medium to 1.5 seconds, and fast to 0.8 seconds
- Faced a bug where the object would teleport at first and then animate normally—it was because I forgot to update the calculation to match the new dynamicSteps
- Fixed that and now the animation speed updates correctly
- Wanted more drastic differences, changed time intervals to Slow = 5 sec and Fast = .5 sec

- Realized the code was getting hard to read, so I had it reorganized and labeled
- Grouped related parts together (like UI setup, animation functions, and object control)
- Added small comments to explain what each section is doing so it's easier to follow and update later
- Had to rearrange Load Preset and Reset Object buttons to after their respective call functions are declared in order for the code to run

- When I tried to run the GUI with the dinosaur preset instead of the person, the object was extremely slow
- Looked into this
- Found the issue in the image file's resolution being much larger, which made it appear to move slower due to its size.
- Looked into how to fix this
- Fixed it by resizing the image when loading to ensure all objects behave consistently regardless of original resolution.


July 8th, 2025

- Goal for today is to work on the changes in Safwans email

- Need to move the buttons and switches to the right side
- Looked into how to do this
- Found a way to move the buttons to the right side
- They appear now horizontally, which makes the GUI super ugly
- Looked into how to make them stack vertically
- Found out the reason was I was using pack(side=tk.LEFT) on every control, which stacks them sideways
- Learned that removing side=tk.LEFT or replacing it with anchor="w" and fill=tk.X helps stack controls vertically
- Reorganized the control layout with .pack(anchor="w", fill=tk.X, pady=5) for nice vertical alignment
- Realized I was referencing variables like bgVar, objVar, speedVar, etc. before they were actually defined
- This caused a NameError, so I needed to move all variable definitions to the top before their first use
- Rearranged the code to define all control variables (StringVar and IntVar) right after setting up the controlFrame
- Went through each control and labeled their function clearly with inline comments (e.g. # --- Object size slider ---)

- Now will further develop the README File
- Wrote a README File explaining how to use the GUI
- Created a separate “sender” script that sends coordinates (points) to the GUI over a network connection on the same computer (localhost)
- Initially ran the GUI, and it printed “waiting for connection from sender” — not sure if this meant it was working or stuck
- Learned the sender script must be run after the GUI so it can connect and send data properly
- Tried running sender script but got errors like “No such file or directory” — realized it was because the command line didn’t recognize the file path correctly
- Figured out that because my folder names had spaces, I needed to put the whole file path inside quotes when running the script from PowerShell
- Once fixed, running sender printed the messages like “Sending: 10,10” — so sender was working
- Ran the GUI again, and now the object smoothly moved to the points sent from sender without me clicking — this was a big success
- Understood that the GUI listens for incoming data in a separate thread so the window stays responsive
- When a new point comes in, it’s added to the same queue that mouse clicks add to, so the animation function automatically moves the object there next
- Noticed a problem where after starting the GUI and sender, the GUI window wouldn’t show properly — it appeared in the taskbar preview but not on screen
- Learned this was probably because the socket listener was blocking or affecting the window’s main loop
- Fixed it by running the socket listener in a background thread so the GUI could keep running and stay visible
- Learned the correct order is: first run the GUI (which listens), then run the sender (which connects)
- Connection requires data to match on both scripts
- Tested the whole setup multiple times, sending multiple points, verifying the object moves smoothly and queues up points in order
- Found the animation speed and smoothness depend on how many steps the animation uses and the timing interval — adjusting these made movement nicer
- Faced alot of problems with file locations, realized I need to be careful with file paths and running scripts from the command line to avoid “file not found” errors




July 9th, 2025



