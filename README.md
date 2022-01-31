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

* `teleop_twist_keyboard (node)`: the teleop_twist_keyboard node is nothing more than a GUI that allows the user to drive a simulated mobile robot by means of the keyboard. In other words, when the user presses certain keys, the node publishes the corresponding velocities on the `cmd_vel` topic made available by the simulator. In this particular case the `/cmd_vel` topic has been remapped in another topic called `/check_vel`, to which the teleop_mediator node is subscribed.

## Software architecture goal
The software architecture goal is to interact with the user and, according to the user request, to control the robot in the environment. In particular, the architecture should make available to the user three different control modalities:
* `1st modality`: autonomous navigation to a user-defined target position
* `2nd modality`: keyboard-guided navigation
* `3rd modality`: keyboard-guided navigation (with assistance in order to avoid collisions)

Concerning this last modality, the robot should not go forward if there is an obstacle in front of it; should not turn left/right if there are obstacles on its left/right. 

## Implementation - description
After the launch of the software architecture two terminal windows are spawned: the first one running the teleop_twist node, the other one running both the robot_gui node and the teleop_mediator node.  
The gui_node prints on the terminal a simple GUI that shows the modalities legend. Depending on the user input these are the possible scenarios:
* `1`: the first modality is chosen. The user can either enter the character 'r' to come back to the original menu or the character 'y' to issue a target position. In this second case, the gui_robot node asks for the coordinates of the position and sends it to the move_base node via the `/move_base/goal` topic. Once the position has been inserted, the user returns to the initial selection (insert 'r' or 'y').
* `2`: the second modality is chosen. A request of the custom service `ChangeMod` containing the string "2" is issued by the robot_gui node that acts as a client. Once the server (teleop_mediator node) has received it, it switches to the second modality. This simply means that it retrieves the velocities published by the teleop_twist_keyboard node on the remapped topic `/check_vel` and, in its turn, publishes them on the `/cmd_vel` topic. In this case the velocities are simply forwarded and no modification is made. The user can return to the selection of the modality by simply entering 'r' in the robot_gui node at any time.
* `3`: the third modality is chosen. A request of the custom service `ChangeMod` containing the string "3" is issued by the robot_gui node that acts as a client. Once the server (teleop_mediator node) has received it, it switches to the third modality. As in the previous case the user can return to the selection of the modality by simply entering 'r' in the robot_gui node at any time.  
The third modality implies that the server node carries out the following tasks:
  * it retrieves the velocities published by the teleop_twist_keyboard node on the remapped topic `/check_vel`.
  * either if there are no obstacles in the way or if there are obstacles but the robot is instructed to go in another direction, the node publishes the retrieved velocities on the `/cmd_vel` topic (the information about the obstacles is obtained from the `/scan` topic).
  * otherwise, if there are obstacles in the way and the robot is instructed to go against them, the node publishes both a null linear velocity and a null angular velocity on the `/cmd_vel` topic (the information about the obstacles is obtained from the `/scan` topic).  
* `q`: the robot_gui node exits and all the other nodes with it, thanks to the `required` attribute associated to it in the launch file

If the string entered is not among these commands, the request for an input is reiterated.  
In order to better understand the structure of the control architecture, the following simplified scheme is provided:  

![Assignment_3-2](https://user-images.githubusercontent.com/91536387/151792336-aff06d4e-4d9e-4d73-a6de-0e9cb28bb80f.png)

## Implementation - GUI node code
The Python script related to the GUI node is composed of a main function and 3 auxiliary functions. The first two auxiliary functions refer to the publisher task carried out by the node, respectively with respect to the `/move_base/goal` topic and to the `/move_base/cancel` topic. The third one refers instead to the client task related to the custom service `ChangeMod` accomplished by the node.

### Main
The main function can be described in pseudocode as follows:
```python
def main ():
	initialize the node with the name "robot_gui_node"
	print the instructions to interact with the node (modalities legend) on the screen
	
	while true:
		ask the user for a modality or "q" to exit
		retrieve the string entered by the user on the stdin

		if the entered string is "1":
			while true:
				ask the user to either insert "y" (for issuing a new target position) or "r" (for retunring to the modality selection)
				retrieve the string entered by the user on the stdin
				
				if the entered string is "y":
					while true:
						ask the user for the coordinates of the target position
						retrieve the strings entered by the user on the stdin
						if the coordinates are numbers 
							break the loop
						else:
							print a warning message on the screen
					call the set_goal_position() function passing as arguments the coordinates of the entered target position 

				if the entered string is "r":
					call the cancel_goal_position() function
					break the loop
		
		if the entered string is either "2" or "3":
			while true:
				call the switch_to_mod() function passing as an argument the string entered by the user
				suggest to the user either to interact with the teleop_twist_keyboard node or to insert "r" (for retunring to the modality selection)
				
				if the entered string is "r":
					call the switch_to_mod() function passing as an argument the string "0"
					break the loop
		
		if the entered string is "q":
			break the loop
	
	quit the node
```

### Auxiliary functions
The first auxiliary function can be described in pseudocode as follows:
```python
def set_goal_position(x, y):
	make use of a message of type MoveBaseActionGoal defined as a global variable
	
	within the message set the frame with respect to which the target position is defined to map frame
	within the message set the orientation of the target position to 1
	within the message set the x coordinate of the target position to the passed x
	within the message set the y coordinate of the target position to the passed y
	
	publish the message
```

The second auxiliary function can be described in pseudocode as follows:
```python
def cancel_goal_position():
	make use of a message of type GoalID defined as a global variable
	
	within the message set the id of the goal position to cancel to "" (empty)
	
	publish the message
```

The third auxiliary function can be described in pseudocode as follows:
```python
def switch_to_mod(num):
	check if the server related to the "change_mod" custom service is up and running
	call the server by issuing a request message containing num (number of the desired modality in form of a string)
	if an exception is raised during the call
		print a warning message on the screen
```

## Implementation - GUI node code
The C++ script related to the controller node is composed of a main function, 2 call-back functions and 2 "regular" functions. The first call-back function refers to the server task carried out by the node and is called every time that a request message belonging to the service `/change_vel` is issued by the client. The second one refers instead to the subscriber task accomplished by the node and is called every time that a message is published on the `/base_scan` topic.

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
