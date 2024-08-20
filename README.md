Section 1   Topics
In this part of the project, you will practice using topics to control a robot. Your goal is to create a ROS program that makes the robot have a wall-following behavior.

<h1 align="center";> </h1>

1.1   Wall following behavior
Remember that you can check the ROS2 Basics course clicking on the notebooks icon in the bottom menu bar.

The wall following behavior is a behavior that makes the robot follow along the wall on its right hand side. This means that the robot must be moving forward at a 30cm distance from the wall, having the wall on its right hand side, the entire time.

To achieve this behavior in the robot, you need to do:

Create a ROS2 package named wall_follower

This package is the one that will contain the full project
Inside the package include a ROS Python file named wall_following.py
Subscribe to the laser topic of the robot

Inside the wall_following.py, you need to subscribe to the laser topic and capture the rays
In the callback of the subscriber, select the ray on the right (the one that makes a 90º angle to the right with the front of the robot) and use it to know the robot's distance to the wall

NOTE: The topics for different simulations most likely will not be named the same. So make sure that you are using the correct name. For example, you used /kobuki/laser/scan during the course. The laser topic in this simulation is /scan

Publish to the velocity topic of the robot

Also inside the wall_following.py, create a publisher to the /cmd_vel topic that controls the wheels
At every step of the control loop, you need to publish the proper velocity command on that topic, based on the value of the distances detected by the laser:

If the distance to the wall is bigger than 0.3m, you need to make the robot approach the wall a little, by adding some rotational speed to the robot
If the distance to the wall is smaller than 0.2m, you need to move the robot away from the wall, by adding rotational speed in the opposite direction
If the distance to the wall is between 0.2m and 0.3m, just keep the robot moving forward
IMPORTANT

When the robot is moving along a wall, it can reach the next wall just infront of it. At that point in time, you should take into account how to progressively transition the robot from following the current wall to the next one.

To detect the wall in the front, we recommend that you use the laser ray just in the front of the robot. If the distance measured by that ray is shorter than 0.5m, then make the robot turn fast to the left (moving forward at the same time).

The result of this whole behavior must be that the robot moves along the whole environment (see video below).

Create a launch file named start_wall_following.launch.py
<h1 align="center";> </h1>

1.2   Test your program in the simulation
Create the wall-following program and test it on the simulation.

If the program doesn't work on the simulation, it is 100% not going to work on the real robot.

Real-life robot development works this way: first, test your program in a simulation. When it works there, try it on a real robot. Never the other way around.



Follow the steps below to test your program in the simulation:

Make sure the simulation is launched, as explained above

In another terminal, launch the keyboard teleop with the following command:

   Execute in a Shell

ros2 run teleop_twist_keyboard teleop_twist_keyboard
Now use the keys to move the robot to a convenient position to test your ROS program. This means, close to a wall, and with the wall on the robot right hand side
IMPORTANT-1: Remember that in order to be able to move the robot with the keys, the terminal where you launched the teleop has to have the focus.

IMPORTANT-2: close this teleop program once you have the proper position. Otherwise, it will interfere with your program.

Now launch your ROS program and see the results. If the robot doesn't behave correctly, debug your program and try to figure out the reason.
<h1 align="center";> </h1>

1.3   If program works in simulation, then go to the real robot
Once the robot is moving along the walls of the simulated environment, it is time for you to test it on the real robot.

The steps are:

Book a date and time in the Real Robot Lab to test your program (see Appendix below that teaches how to do that).
On the day and time selected, open this rosject
Connect to the real robot lab from within this rosject (see Appendix below)
Use the web-joystick that appears on the cameras window and move the real robot to the proper position to start before you launch your program
You don't need to launch the simulation, because you are using the real thing
Launch your program and see if it works properly. Chances are it will not because simulation is not an exact copy of the real environment. Now is your time to debug
Test on the real robot and debug your errors until you have it working as intended


Section 2   Services
In the previous section, you used the keyboard to put the robot at a proper position so it can do the follow wall behavior.

In this section you will create a program that does that alignment automatically.

The goal in this section is to create a new program that when called,

Makes robot search for the nearest wall
Once found, the robot will approach to the proper distance (0.3 m)
And then aligns itself with the wall on its right hand side, waiting in that position, ready to do the wall following behavior
Check this video, to understand the behavior that is required:



Robot behavior required
The robot of the video above does the following:

Because of the 360º laser, the robot can detect the closest wall (shortest laser ray)
Once identified the closest wall, the robot rotates to face that wall
Once the robot is facing that wall, it moves closer to it, until the robot is at 0.3 m of it
Then, the robot rotates again so it ends with the wall to its right hand side
We will call that behavior, the find wall behavior.

The find wall behavior must be provided by a service server. What does it mean?

You have to create a new node with a service server called find_wall
When somebody calls that server, the server callback function must execute the code of the find wall behavior, and make the robot find the closest wall and align with it
This server must be called before starting the wall following behavior.

To accomplish all that, you will have to do three things:

Create a service server node that, when called, will make the robot do the find wall behavior
Modify the program from Section 1 so that it includes now a service client that calls the service server before starting the wall following behavior
Create a new launch file to launch both nodes (wall following node and the node with the find_wall service server)
LET'S GOT FOR IT!

<h1 align="center";> </h1>

2.1   Create Service Server
Inside the wall_follower package create a new ROS file named wall_finder.py
Inside wall_finder.py create a service server named find_wall

The callback of the server must contain the code that performs the find wall behavior

Some tips for that

Identify which laser ray is the shortest one. Assume that this is the one pointing to a wall
Rotate the robot until the front of it is facing the wall. This can be done by publishing an angular velocity until front ray is the smallest one
Move the robot forward until the front ray is smaller than 30cm
Now rotate the robot again until ray number 270 of the laser ranges is pointing to the wall
At this point, consider that the robot is ready to start following the wall
Return the service message with True
The server must use a new FindWall interface that you must also create, and that has the following structure:
FindWall.srv

---
bool wallfound
Check sections 4.6 and 4.7 to remember how to do this part.

Create a launch file named start_wall_finder.launch.py that launches the wall_finder.py
<h1 align="center";> </h1>

  Test Service Server
How can you test and debug the service server independently of the code of Part 1? By doing the following steps:

Launch the service server node using start_wall_finder.launch.py
Put the simulated robot in a proper location for your test. You can do that using the keyboard_teleop explained in the previous section
Call the service from a terminal by issuing the command:
   Execute in Terminal #2

ros2 service call /find_wall std_srvs/Empty '{}'
This should start the robot's movement looking for the wall. If it doesn't, it means your program doesn't work properly. You need to debug!

<h1 align="center";> </h1>

2.2   Add server call to Section 1 program
Once the service is working, you need to call it from the wall_following.py program that you created in section 1.

The idea is that before the wall following behavior starts following the wall, the robot has to be already aligned to the wall. Hence, the wall_following.py must call the /find_wall service before starting doing the wall following behavior.

So you need to do the following:

Add to the wall_following.py node a synchronous service client that connects to the /find_wall service
Use that client to call the service before the control loop of that node, so the robot gets prepared autonomously before following the wall
<h1 align="center";> </h1>

2.3   Create new launch file
Now you need to create a launch file that launched both the wall_following.py and the wall_finder.py, because both of them have to be running in order to work together.

Create a new launch file named main.launch.py that launches both nodes: first the service server node and then the wall following node

Put the robot in the simulation at a proper location for testing

Then call the main.launch.py to start the full program

This should start the robot movement (first the robot looks for the wall, then it starts to follow it).

IMPORTANT: Test everything on the simulation. When it works in the simulation, book a date in the Real Robot Lab and make it work there, too. Same as in Section 1.

Section 3   Actions
Create an action server that records odometry
The goal of this section is to create an action server that records the robot's odometry while the robot is moving along the wall, so we can later use it to see the path followed by the robot.

To achieve that, you will need to create another node that contains an action server in charge of doing the odometry recording when the action server is activated.

The steps required to achieve that are:

Create an action server that, when called, starts recording odometry
Add a call to the action server from the wall following node
Include the launch of the action server in main.launch.py
<h1 align="center";> </h1>

3.1   Create action server
You need to do the following things:

The server must start recording the (x, y, theta) odometry of the robot as a Point32, one measure every second
As feedback, the action server must provide the total amount of meters that the robot has moved so far
When the robot has done a complete lap, it has to finish and return the list of odometries recorded
To achieve that, make the following steps:

Create a new ROS node in a new Python file named odom_recorder.py
odom_recorder.py must have a subscription to the /odom topic. The callback of the subscriber must:
Get the last odometry value received and extract the x, y
Save those values on a geometry_msgs/Point variable of the class named self.last_odom
Include in odom_recorder.py an action server named /record_odom
Inside the callback of the action server, you need to do the following:
Get the first odometry value received and store in a variable named self.first_odom
Then every second, get the last odometry value of self.last_odom and store in an array of Point values named self.odom_record
Inside the Feedback callback, you need to do the following:
Have a variable with current total distance named self.total_distance
Have another variable with last (x,y) position, named self.last_x and self.last_y
At every step of the feedback, compute the distance between (self.last_x, self.last_y) and (self.last_odom.x, self.last_odom.y)
Then add that distance to self.total_distance
Return as feedback the current value of self.total_distance
At every step, check that the distance between (self.last_odom.x, self.last_odom.y) and (self.first_odom.x, self.first_odom.y) is less than 5 cm
If less than 5 cm, then it means that the robot has reached the initial position again, so the action server must end and return as result the total list of (x,y) points that were stored in the self.odom_record array
This server uses an OdomRecord.action message that you must also create:
   OdomRecord.action

---
geometry_msgs/msg/Point[] list_of_odoms
---
float32 current_total
<h1 align="center";> </h1>

3.2   Add action service client to Section 1 program
Once the action server is working, you need to call it from the wall_following.py program that you created in section 1.

The idea is that before the wall following behavior starts following the wall, the wall_following.py must call the /record_odom action to indicate that the recording of the odometry must start.

So you need to do the following:

Add to the wall_following.py node an action client that connects to the /record_odom action server
Use that client to call the action before the control loop of that node, so it starts recording the odometry before following the wall
<h1 align="center";> </h1>

3.3   Add action server to main launch file
Now you need to modify the main.launch.py to include the launch of the odom_recorder.py.

Modify the main.launch.py launch file to launch the three nodes: first the service server node, then the action server node, and then the wall following node

Put the robot in the simulation at a proper location for testing

Then call the main.launch.py to start the full program

After the robot starts moving you should see the feedback appearing
