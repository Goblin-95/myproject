This project is a Python-based graphical interface that animates an object moving across a canvas. The object can be moved either by clicking on the canvas or by receiving live target coordinates from a separate sender script running in the background. The GUI is built using Tkinter and Matplotlib, and now includes size, speed, background, and character selection controls.

How to use:

Step 1: Open a terminal window and run the Animating Sender script first

Step 2: In a second terminal, run the GUI Receiver script to open the visual interface.

Once both are running, you can now click on the interface. The object will smoothly move to the point you clicked, continuing to move as you click more. You can also change the speed and size of the object using the controls on the right.


What each script does:

- gui_receiver.py launches a visual interface where:
- You can click anywhere to send a target location to the sender.
- Users can choose preset backgrounds and characters using dropdowns.
- There is a live slider for object size.
- A dropdown lets you select animation speed: slow, normal, or fast.
- A “Center Character” button resets the object to the center of the canvas.
- Starts a background thread to listen for live position updates from the sender.
- When it receives position data, it moves the object smoothly using imshow.
- animating_sender.py is a background script that:
- Connects to the GUI over a local socket (127.0.0.1).
- Listens for coordinate messages like "x,y,speed".
- Animates the object by sending incremental updates from its current position to the new target.
- Adjusts speed by changing how many steps and how fast each one is sent:
- Slow = 70 steps with delay
- Normal = 50 steps
- Fast = 30 steps with shorter delay

How the scripts communicate:

- The two scripts use Python sockets over localhost:
- The sender connects first and waits.
- When the GUI launches, it accepts the sender's connection.
- When the user clicks, the GUI sends a message like "40.3,60.7,fast" to the sender.
- The sender processes that and sends back a series of intermediate positions.
- The GUI listens for those and animates the object’s movement accordingly.