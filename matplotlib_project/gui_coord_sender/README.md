This project is a Python-based graphical interface that animates an object moving across a canvas. The object can be moved either by clicking on the canvas or by receiving live target coordinates from a separate sender script running in the background. The GUI is built using Tkinter and Matplotlib, and now includes size, speed, background, and character selection controls.

The second part of this project allows for connection to a TurtleBot in an Ubuntu virtual machine that allows for the movement of the TurtleBot to be plotted onto the GUI. Scroll down all the way to see instructions on how to do this. 

How to use:

Step 1: Open the `gui_receiver` file
Step 2: Run the `animating_sender` file in the terminal
Step 3: Choose your presets
Step 4 (optional): If you run the GPS background option, include the coordinates of what area youd like visualized as a background on the GUI, as well as the level of zoom you would like to have on the GPS background. (UIC Coordinates: 41.8708, -87.6505)
Step 5: Apply your presets

Once both scripts are running, you can now click on the interface. The object will smoothly move to the point you clicked, continuing to move as you click more. You can also change the speed and size of the object using the controls on the right. Clicking the "Center Object" option will bring the 


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

----------------------------------------------------------------------------------------------------------------------------------
Run Ubuntu TurtleBot + GPS:

Uses:

- Ubuntu 22.04 VM through VirtualBox
- ROS2
- TurtleBot3\
- VS Editor 

Run these commands in your Ubuntu VM in 4 different terminals in order to simulate the robot, move it, and log its GPS coordinates. 

Terminal 1: Launch Gazebo

source /opt/ros/humble/setup.bash
cd ~/turtlebot_gps_workspace
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch gazebo_ros gazebo.launch.py gui:=false

Terminal 2: Spawn Robot

source /opt/ros/humble/setup.bash
cd ~/turtlebot_gps_workspace
export TURTLEBOT3_MODEL=waffle_pi
ros2 run gazebo_ros spawn_entity.py -file ~/turtlebot_gps_workspace/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -entity turtlebot3 -x 0 -y 0 -z 0.01

Check topics: 

source /opt/ros/humble/setup.bash
cd ~/turtlebot_gps_workspace
export TURTLEBOT3_MODEL=waffle_pi
ros2 topic list

Terminal 3: Auto Logger

cd ~/turtlebot_gps_workspace
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
python3 auto_gps_logger.py


Terminal 4: Turtlebot Movement


Teleop:
cd ~/turtlebot_gps_workspace
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Move forward:

cd ~/turtlebot_gps_workspace
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}" --once


To connect to your GUI:
- Run `gui_receiver` 
- Run `windows_gps_bridge` through the terminal, 
- Select option 2

You should now see the object in the GUI move accordingly to the movement of the TurtleBot. 

