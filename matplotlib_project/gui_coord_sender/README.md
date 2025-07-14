## Coordinate GUI Point & Click ##

This project is a Python-based graphical interface that animates an object moving across a canvas. The object can be moved either by clicking on the canvas or by receiving target coordinates from a separate script running in the background. The GUI is built using Tkinter and Matplotlib, with real-time animation support.

How to use:

Step 1: Run the GUI Receiver script to open the visual interface, it will now begin to listen for the incoming data
Step 2: Open a new terminal
Step 3: Run the Animating Sender script using the new terminal and the path of the file
as so : python "C:\Users\georg\python_projects_all\matplotlibPROJECT\matplotlib_project\gui_coord_sender\animating_sender.py"

After these steps, you can now click on the interface and the object will move to your target and will continue to do so.


What each script does:

gui_receiver.py launches a visual interface where:

- Users can click to move an image/object on a background.
- Users can choose preset backgrounds and characters.
- The object animates between target points.
- Starts a socket listener in the background to receive coordinate data from another script.
- When it receives data, it queues the coordinates and animates the object to move there smoothly.

animating_sender.py is asimple script that:

- Connects to the GUI over a local network socket (127.0.0.1):
- Sends a predefined list of target coordinates (e.g., (10, 10), (30, 30), (70, 70)) to the GUI.
- Simulates an external controller or input source sending real-time movement instructions.


How the scripts communicate:

The communication between the two scripts is done through Python sockets on the same machine (localhost).
The GUI (gui_script.py) opens a socket and waits for incoming connections on a specific port (e.g., 65432).
The sender script connects to that port and sends coordinate strings like "10,10", "30,30", etc.

When the GUI receives a new coordinate:

- It converts the string to a pair of floats.
- It queues that position.
- It animates the object to move there.


Notes:

- Make sure both scripts are using the same IP address and port number (both should match).
- Run the GUI before the sender, or the sender will fail to connect.
- You must have the required images in the proper backgrounds/ and objects/ folders as referenced in the GUI code.
- Requires Python libraries: matplotlib, tkinter, PIL (Pillow), and numpy. 

