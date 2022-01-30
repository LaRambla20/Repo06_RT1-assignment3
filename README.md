# Third Reasearch Track 1 Assignment
The repository in which this README is contains a ROS package, named `final_assignment_controller` and composed by four folders, a text file and a file with extension .xml: 

* `include`: folder for the management of included packages (not used)
* `launch`: folder that contains two launch files (`final_assignment_sim.launch` and `control_architecture.launch`): one whose aim is to launch the simulation environment and both the SLAM and path planning algorithms; the other whose aim is to launch the implemented control architecture
* `scripts`: folder that contains two python scripts (`robot_gui.py` and `teleop_mediator.py`) realizing two nodes: one whose aim is to interact with the user, to implement the first modality for driving the robot and to send requests to the second node; the other whose aim is to implement the second and third modality for driving the robot under request
* `srv`: folder that contains a custom ROS service (`ChangeMod.srv`) whose aim is to make the two previously-mentioned nodes communicate
* `CMakeLists.txt`: text file that describes how to build the code and where to install it to
* `package.xml`: XML file that defines properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages

## How to install and run
In order to make use of the package contained in this repository, other two packages are needed. Those are:
* `slam_gmapping` package: which can be obtained by cloning in the `src` folder of your ROS workspace the following github repository https://github.com/CarmineD8/slam_gmapping (after cloning, remember to checkout to the branch corresponding to your ROS distribution)
* `final_assignment` package: which can be obtained by cloning in the `src` folder of your ROS workspace the following github repository https://github.com/CarmineD8/final_assignment (after cloning, remember to checkout to the branch corresponding to your ROS distribution)  

It is also necessary to check if the `ROS navigation stack` is installed in your machine. If not, run the following command from terminal:
```bash
$ sudo apt-get install ros-<your_ros_distribution>-navigation
```
The same thing holds for the `teleop_twist_keyboard` package. If it is not installed, run the command:
```bash
$ sudo apt-get install ros-<your_ros_distribution>-teleop-twist-keyboard
```

That been said, as far as the package contained in this repository is concerned, it can be obtained by simply cloning the current repository in the `src` folder of your ROS workspace (no checkout to other branches is needed).

Once that all the required packages has been downloaded correctly, execute the following line to build your ROS workspace:
```bash
$ catkin_make
```
Regarding the execution, two launch files has been written:
* `final_assignment_launch.launch`: launch file, contained in the implemented package, that simply includes two launch files from the auxiliary package `final_assignment`(`simulation_gmapping.launch` and `move_base.launch`). The first one of these two runs both the simulation (in Gazebo) and the SLAM algorithm (gmapping), the second one runs the path planning algorithm (move_base).
* `control_architecture.launch`: launch file, contained in the implemented package, that runs three nodes: `robot_gui.py` and `teleop_mediator.py`, that are the two developed nodes, and `teleop_twist_keyboard_node`, that is part of the previously installed package. The last node is run in a separate terminal window.

In order to launch these launch files:
* open a terminal window, navigate to your ROS workspace and run the following line:
```bash
$ roslaunch final_assignment_controller final_assignment_sim.launch
```
* open another terminal window, navigate to your ROS workspace and execute:
```bash
$ roslaunch final_assignment_controller final_assignment_sim.launch
```
### About the simulator and the algorithms
This assignment relies on some software facilities, the most relevant of which are:
* `Gazebo`: Gazebo is an open-source 3D robotic simulator integrated into ROS that has the ability to accurately and effciently simulate populations of robots in complex indoor and outdoor environments. In the considered case, a 3D mobile robot is spawned inside an indoor environment composed of different rooms. The rooms and the environment are delimited by walls.  
For controlling the robot the simulator makes some topics and services available. As regards the topics, two of them are mainly used in the proposed solution:
  * `/scan`: topic on which the simulation node publishes the output of the robot laser scanners. The type of message sent on this topic is `sensor_msgs/LaserScan` and consists in several fields. Among them, the `ranges` field, which is a vector that contains the distances of the robot from the walls in an angular range from 0 to 180 degrees, will be exploited.
  * `/cmd_vel`: topic to which the simulation node is subscribed in order to receive commands to set the robot linear and angular velocity. The type of message sent on this topic is `geometry_msgs/Twist` and consists in two fields. They both are three dimensional vectors, but one specifies the linear velocity of the robot, the other its angular velocity. Among the elements of these two vectors the `x` component of the linear vector and the `z` component of the angular vector will be used in order to guide the robot along the circuit.

* `Rviz`: Rviz is a 3D visualization tool for ROS that interacts with Gazebo
* `slam_gmapping (node)`: the slam_gmapping node implements a common SLAM (Simultaneous Localization And Mapping) algorithm, based on the continuous evaluation of mass probability functions.
* `move_base (node)`: the move_base node implements a path planning algorithm that aims at accomplishing global navigation tasks by linking together a global and local planner. Since, in mobile-robots navigation, reaching a target position is not an istantaneous task, the move_base node makes use of an action to allow external nodes to interact with it. Actions can be seen as collections of 5 different topics characterized by the same name space: goal topic, cancel topic, status topic, result topic, feedback topic. In the considered case, the move_base node acts like an action server, and two out of the five topics are exploited in order to set or cancel a goal position. As the names suggest, these are `/move_base/goal` and `/move_base/cancel`.
  * `/move_base/goal`: topic to which the move_base node is subscribed in order to receive commands to set a new goal position. The type of message sent on this topic is `move_base_msgs/MoveBaseActionGoal` and consists in several nested fields. Among them the ones of interest are: `goal.target_pose.frame_id` (containing the frame with respect to which the target position is defined), `goal.target_pose.pose.orientation.w` (containing the target orientation), `goal.target_pose.pose.position.x` (containing the target-position x coordinate), `goal.target_pose.pose.position.y` (containing the target-position y coordinate).
  * `/move_base/cancel`: topic to which the move_base node is subscribed in order to receive commands to cancel a goal position. The type of message sent on this topic is `action_lib_msgs/GoalID` and consists in two fields. One of them in particular is used. Its name is `id` and contains the id of the goal position to be cancelled.

* `teleop_twist_keyboard (node)`: the teleop_twist_keyboard node is nothing more than a GUI that allows the user to drive a simulated mobile robot by means of the keyboard. In other words, when the user presses certain keys, the node publishes the corresponding velocities on the `cmd_vel` topic made available by the simulator. 

## Nodes goal
As stated above, the simulation environment opened by the `stageros` node is a circuit delimited by walls. The simulated robot is spawned inside this perimeter near its bottom-right corner. Among the two implemented nodes, the goal of the controller node is to guide the robot along the path in clockwise direction, making sure that it does not collide with the walls. Furthermore it should meet the requests of the other node and send to it responses. Whereas the GUI node is aimed at constantly asking and waiting for an input from the user, which can either ask to increment or decrement the velocity, or to put the robot in the initial position. Once an input has been detected, it should then issue a correspondent request to the first node. The final behaviour of the robot should therefore be to autonomously drive between the walls and to increment/decrement its velocities or resetting its position under the request of the user.

## Implementation - description
Since the primary goal of the robot is to avoid colliding with the walls, first of all the robot should check if there are walls at a dangerous distance. Then, if the answer is yes, it will be instructed to change the direction of its motion according to the position of the critical obstacle.  
As far as the detection task is concerned, the output of the laser scanner is used. As mentioned above, this information is sent on the `/base_scan` topic by the simulation node. The controller node then subscribes to the topic at issue and continuously checks the messages sent on it. In particular the field of interest is named `ranges` and contains 720 elements that are the distances between the robot and the walls in an angular range that goes from 0 to 180 degrees. Given the great amount of data, a basic processing of them is carried out. In order to do that, the vector is divided in 5 sub-vectors, each one defining a different region:
* `Region 1 - right region`: region that ranges from 0 to 36 degrees
* `Region 2 - front-right region`: region that ranges from 36 to 72 degrees
* `Region 3 - front region`: region that ranges from 72 to 108 degrees
* `Region 4 - front-left region`: region that ranges from 108 to 144 degrees
* `Region 5 - left region`: region that ranges from 144 to 180 degrees

This division is shown in the following picture:  
![Robot_RT1_2](https://user-images.githubusercontent.com/91536387/145257341-ecc88710-b918-49a9-a9f0-21e75f4e2f3f.png)  
That been done, it is convenient to identify a representative distance for each region. For this reason the minimum distance among all the distances of each region is extracted. The five retrieved values are then used to check the presence of dangerous walls within the visual field of the robot and eventually to avoid them. In particular, according to the distances belonging to the three frontal regions (`front`, `front-left` and `front-right` area), different states are defined, each one related to a different action. Critical states are then further divided in substates, by checking the two side areas:
* `state 1 - "nothing"`: occurs if no wall is within dangerous distance. The robot is instructed to go forward.
* `state 2 - "front"`: occurs if the minimum distance belonging to the front region is less than the threshold `DAN_DISTANCE`. Since given this information it is not really clear where the robot should turn, the side areas are checked as well and three substates are defined:
  * `substate a - "right" `: occurs in the second state if the minimum distance belonging to the right region is less than the threshold `DAN_DISTANCE`. The robot is instructed to turn left.
  * `substate b - "left" `: occurs in the second state if the minimum distance belonging to the left region is less than the threshold `DAN_DISTANCE`. The robot is instructed to turn right.
  * `substate c - "left and right" `: occurs in the second state if both the minimum distance belonging to the right region and the minimum distance belonging to the left region are less than the threshold `DAN_DISTANCE`. The robot is instructed to turn left.

* `state 3 - "front-right"`: occurs if the minimum distance belonging to the front-right region is less than the threshold `DAN_DISTANCE`. The robot is instructed to turn left.
* `state 4 - "front-left"`: occurs if the minimum distance belonging to the front-left region is less than the threshold `DAN_DISTANCE`. The robot is instructed to turn right.
* `state 5 - "front and front-right"`: occurs if both the minimum distance belonging to the front region and the minimum distance belonging to the front-right region are less than the threshold `DAN_DISTANCE`. The robot is instructed to turn left.
* `state 6 - "front and front-left"`: occurs if both the minimum distance belonging to the front region and the minimum distance belonging to the front-left region are less than the threshold `DAN_DISTANCE`. The robot is instructed to turn right.
* `state 7 - "front and front-right and front-left"`: occurs if the minimum distances belonging to all the three frontal regions are less than the threshold `DAN_DISTANCE`. Since given this information it is not really clear where the robot should turn, the side areas are checked as well and three substates are defined:
  * `substate a - "right"`: occurs in the seventh state if the minimum distance belonging to the right region is less than the threshold `DAN_DISTANCE`. The robot is instructed to turn left.
  * `substate b - "left"`: occurs in the seventh state if the minimum distance belonging to the left region is less than the threshold `DAN_DISTANCE`. The robot is instructed to turn right.
  * `substate c - "left and right"`: occurs in the seventh state if both the minimum distance belonging to the right region and the minimum distance belonging to the left region are less than the threshold `DAN_DISTANCE`. The robot is instructed to turn left.

The linear and angular velocities that are needed to accomplish all these tasks are published by the controller node on the topic `cmd_vel`. The `stageros` node is then responsible for setting them in the simulation.  
Finally, since the `robot_controller_node` already has access to the `cmd_vel` topic, it is also instructed to act as a server with respect to a service aimed at meeting the requests of the user. In particular, by modifying two coefficients under request (`coeff_l` and `coeff_a`), it can:  
* start the motion
* increment/decrement the robot velocities
* reset both the robot position and the velocities.

The service used to this end is not made available by the simulation node, so it is created. Its structure consists in a request message made of a char variable (`command`) and a response message made of a string variable (`action`).  
Before passing on to the GUI node, a representation of the controller node and of the communication channels that connects it with the simulation node is hereafter shown:  
![Rqt_Graph_cut](https://user-images.githubusercontent.com/91536387/145423031-db96b237-c835-47a8-bc22-20a73fab4720.png)  
Regarding the `robot_gui_node`, its operating principle  is very simple. It asks the user for a command and waits until something is provided. The recognized commands are:
* `s`: start the robot motion
* `d`: increment the robot linear velocity
* `a`: decrement the robot linear velocity
* `c`: increment the robot angular velocity
* `z`: decrement the robot angular velocity
* `r`: reset the robot position and velocities
* `q`: quit the GUI node

If the string entered is not among these commands, a warning message is issued on the screen. Otherwise, for the first six elements of this list the node sends a request message containing the correspondent character to the server node and prints the latter's response. These actions are ciclically repeated until the command `q`, which makes the node terminate, is entered by the user.

## Implementation - controller node code
The C++ script related to the controller node is composed of a main function, 2 call-back functions and 2 "regular" functions. The first call-back function refers to the server task carried out by the node and is called every time that a request message belonging to the service `/change_vel` is issued by the client. The second one refers instead to the subscriber task accomplished by the node and is called every time that a message is published on the `/base_scan` topic.

### Main
The main function can be described in pseudocode as follows:
```cpp
int main (arguments vector dimension, arguments vector){
	initialize the node with the name "robot_controller_node"
	setup the NodeHandle
	initialize the publisher that publishes on the "/cmd_vel" topic
	initialize and define the subscriber that subscribes to the "/base_scan" topic and assign the "robotCallback" call-back function to it
	initialize and define the server that answers to requests belonging to the "/change_vel" service and assign the "obtaincoeffCallback" call-back function to it
	spin to allow the call-back functions to be called whenever a message arrives on the correspondent topic or service
}
```

### Call-back functions
The first callback function can be described in pseudocode as follows:
```cpp
bool obtaincoeffCallback (request message related to the service "/change_vel", response message related to the service "/change_vel"){
	if the request message is the command "s"
		if both cefficients are 0
			assign 1 to both the coefficents
			fill the response message with "motion started"
		else
			fill the response message with a warning

	else if the request message is the command "d"
		if the coefficient related to the linear velocity is 0
			fill the response message with a warning
		else
			multiply the coefficient related to the linear velocity by 1.5
			fill the response message with "linear velocity incremented"

	else if the request message is the command "a"
		if the coefficient related to the linear velocity is 0
			fill the response message with a warning
		else
			divide the coefficient related to the linear velocity by 1.5
			fill the response message with "linear velocity decremented"

	else if the request message is the command "c"
		if the coefficient related to the angular velocity is 0
			fill the response message with a warning
		else
			multiply the coefficient related to the angular velocity by 1.2
			fill the response message with "angular velocity incremented"

	else if the request message is the command "z"
		if the coefficient related to the angular velocity is 0
			fill the response message with a warning
		else
			divide the coefficient related to the angular velocity by 1.2
			fill the response message with "angular velocity decremented"

	else if the request message is the command "r"
		assign 0 to both the coefficents
		fill the response message with "position and velocities reset"
}
```
The second callback function can be described in pseudocode as follows:
```cpp
void robotCallback(message retrieved by the "/base_scan" topic){
	divide the "ranges" field of the message in 5 subvectors, one for each region
	call the "find_minimum" function to retrieve the minimum distance from the walls for each one of the 5 regions
	print on the screen the obtained minimum distances
	call the "change_direction" function to set the velocities of the robot according to the position of the walls at a dangerous distance and to retrieve the current state and substate
	print on the screen the obtained state and substate
	publish on the "/cmd_vel" topic the velocities set by the "change_direction" function call
}
```
### "Regular" functions
The first "regular" function can be described in pseudocode as follows:
```cpp
float find_minimum (vector of floats, size of the vector){
	find the minimum value among the elements of the vector passed as input
	return the retrieved minimum element
}
```
The second "regular" function can be described in pseudocode as follows:
```cpp
vector of strings change_direction (vector of floats, pointer to the linear velocity of the robot, pointer to the angular velocity of the robot){
	initialize the substate description
	
	if there is no obstacle at a dangerous distance
		update the state description
		go straight

	else if there is an obstacle at a dangerous distance in the front region
		update the state description
		if there is an obstacle at a dangerous distance in the right region
			update the substate description
			turn left
		else if there is an obstacle at a dangerous distance in the left region
			update the substate description
			turn right
		else
			turn left

	else if there is an obstacle at a dangerous distance in the front-right region
		update the state description
		turn left

	else if there is an obstacle at a dangerous distance in the front-left region
		update the state description
		turn right

	else if there is an obstacle at a dangerous distance in both the front region and the front-right region
		update the state description
		turn left

	else if there is an obstacle at a dangerous distance in both the front region and the front-left region
		update the state description
		turn right

	else if there is an obstacle at a dangerous distance in both the front region, the front-right region and the front-left region
		update the state description
		if there is an obstacle at a dangerous distance in the right region
			update the substate description
			turn left
		else if there is an obstacle at a dangerous distance in the left region
			update the substate description
			turn right
		else
			turn left

	else if there is an obstacle at a dangerous distance in both the front-right region and the front-left region
		update the state description
		go straight
		
	else
		update the state description

	return a vector containing both the state description and the substate description
}
```

## Implementation - GUI node code
As regards the GUI node instead, its structure is much simpler since it consists in only the main function.

### Main
The main function can be described in pseudocode as follows:
```cpp
int main (arguments vector dimension, arguments vector){
	initialize the node with the name "robot_gui_node"
	setup the NodeHandle
	initialize and define a client ("client1") that sends requests belonging to the "/change_vel" service
	define the custom service variable "srv1" of type "ChangeVel" 
	initialize and define a client ("client2") that sends requests belonging to the "/reset_positions" service
	define the service variable "srv2" of type "Empty" 
	initialize an iterations counter related to the linear velocity of the robot to 0
	initialize an iterations counter related to the angular velocity of the robot to 0

	print the instructions to interact with the node on the screen

	while true
		ask the user for a command
		retrieve the command entered by the user on the stdin

		if the entered command is "s"
			check if the server related to the "/change_vel" service is up and running
			fill the request message of the variable "srv1" with the character "s"
			make the first client call the related service with the variable "srv1"

			print the server response on the screen

		else if the entered command is "d"
			check if the server related to the "/change_vel" service is up and running
			fill the request message of the variable "srv1" with the character "d"
			make the first client call the related service with the variable "srv1"

			if the response is a warning
				print the server response on the screen in red
				assign 0 to the iterations counter related to the linear velocity of the robot
			else
				print the server response on the screen in green
				increment the iterations counter related to the linear velocity of the robot

		else if the entered command is "a"
			check if the server related to the "/change_vel" service is up and running
			fill the request message of the variable "srv1" with the character "a"
			make the first client call the related service with the variable "srv1"

			if the response is a warning
				print the server response on the screen in red
				assign 0 to the iterations counter related to the linear velocity of the robot
			else
				print the server response on the screen in green
				decrement the iterations counter related to the linear velocity of the robot

		else if the entered command is "c"
			check if the server related to the "/change_vel" service is up and running
			fill the request message of the variable "srv1" with the character "c"
			make the first client call the related service with the variable "srv1"

			if the response is a warning
				print the server response on the screen in red
				assign 0 to the iterations counter related to the angular velocity of the robot
			else
				print the server response on the screen in green
				increment the iterations counter related to the angular velocity of the robot

		else if the entered command is "z"
			check if the server related to the "/change_vel" service is up and running
			fill the request message of the variable "srv1" with the character "z"
			make the first client call the related service with the variable "srv1"

			if the response is a warning
				print the server response on the screen in red
				assign 0 to the iterations counter related to the angular velocity of the robot
			else
				print the server response on the screen in green
				decrement the iterations counter related to the angular velocity of the robot

		else if the entered command is "r"
			check if the server related to the "/change_vel" service is up and running
			fill the request message of the variable "srv1" with the character "r"
			make the first client call the related service with the variable "srv1"

			print the server response on the screen
			assign 0 to both the iterations counters

			check if the server related to the "/reset_position" service is up and running
			make the second client call the related service with the variable "srv2"

		else if the entered command is "q"
			exit
		else
			print "invalid command" on the screen

		if the iterations counter related to the linear velocity of the robot is greater than 3
			issue a warning message on the screen
		if the iterations counter related to the angular velocity of the robot is greater than 3
			issue a warning message on the screen

	return
}
```

## System Limitations and Possible Improvements
As far as the limitations of the script are concerned, the main one is that, by increasing the velocities too much, the control of the robot might fail. In the proposed solution the GUI node issues a warning message on the screen if at least one of the velocities is too high. Another way to deal with this problem could be to simply not allow the user to increment the velocities once that they reached a certain value.  
As regards the improvements instead, several of them could be carried out. For instance, one of them could be to allow the user to directly specify the desired velocities by means of the GUI node.
