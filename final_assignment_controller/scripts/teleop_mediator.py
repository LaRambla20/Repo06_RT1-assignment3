#! /usr/bin/env python

"""
.. module:: teleop_mediator
    :platform: Unix
    :synopsis: Python module that implements under request the second and third control modality, by acting as a mediator between the teleop_twist_keyboard node and the simulator

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

This node implements either modality 2 or modality 3. Both modalities are realized by performing a check over the velocity sent by the teleop_twist_keyboard module. 
In particular, in modality 2 the velocity is directly forwarded to the simulator; whereas in modality 3 the forwarding of the desired velocity takes place only if it does not 
endanger the robot. Otherwise the robot is stopped. 
The switch between the two modalities at issue occurs under request of the robot_gui node.

Subscribes to:
    - /scan
    - /check_vel

Publishes to:
    - /cmd_vel

Service:
    - /change_mod

"""

import rospy # import rospy to use ros functionalities
from geometry_msgs.msg import Twist # import the type of message that is exchanged both on the /cmd_vel topic and on the /check_vel topic
from sensor_msgs.msg import LaserScan # import the type of message that is exchanged on the /scan topic
from final_assignment_controller.srv import ChangeMod, ChangeModResponse # import both the request message type and the response message type of the ChangeMod.srv custom service

# Publisher
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # initialize and define the publisher that publishes on the /cmd_vel topic
"""
Global publisher for setting the robot velocity
"""

# GLOBAL CONSTANTS

dan_dist = 0.6
"""
Safety distance
"""

# GLOBAL VARIABLES

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
mod = "0"
"""
Global string variable containing the number of the set modality
"""

# Published message
vel = Twist()
"""
Global message published on the /cmd_vel topic (contains the robot velocity to be set)
"""

# Auxiliary global variables for the GUI aesthetic
i = 0


# SERVICE CALLBACK

# function that is called every time that a new client request related to the /change_mod service is received
def clbk_changemod_srv(req):

    """Function that is called every time that a new client request related to the /change_mod service is received. 

    The number related to the requested modality is stored in a global variable as a string. If the string is "0" the robot is stopped.

    Args:
        req (str): number of the desired control modality stored as a string

    Returns:
        True: boolean constant to warn the client about the correct processing of its request

    """

    global mod
    global vel 

    mod = req.modality
    # print("the requested modality is: " + mod)
    if(mod == "0"):
        vel.linear.x = 0
        vel.angular.z = 0
        pub.publish(vel)
    return ChangeModResponse(True)


# TOPIC CALLBACKS

# function that is called every time that a new message is published on the /scan topic
def clbk_laser(msg):

    """Function that is called every time that a new message is published on the /scan topic.
 
    The robot visual field is divided in 5 regions and the minimum distance balonging to each region is retrieved. Based on this information, if the selected modality is 3, 
    it is checked whether the desired robot velocity can cause a collision with the walls. If the answer is yes, the robot is stopped; otherwise, the desired robot velocity is set.

    Args:
        msg (struct): structure containing the output of the robot's laser scanners

    """

    global regions_
    global vel

    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    if(mod == "3"):

        # rospy.loginfo("\nThe distances in the regions are:\n - front: %f\n - right: %f\n -left: %f ;", regions_['front'], regions_['right'], regions_['left'])
        
        if(regions_['front'] < dan_dist and vel.linear.x > 0):
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)
            print('\033[91m' + "\nFrontal danger of collision" + '\033[0m' + " -> motors stopped")
            print("Change the motion direction or enter 'r' to change modality: ")

        elif(regions_['right'] < dan_dist and vel.angular.z < 0):
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)
            print('\033[91m' + "\nLateral (right) danger of collision" + '\033[0m' + " -> motors stopped")
            print("Change the motion direction or enter 'r' to change modality: ")

        elif(regions_['left'] < dan_dist and vel.angular.z > 0):
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)
            print('\033[91m' + "\nLateral (left) danger of collision" + '\033[0m' + " -> motors stopped")
            print("Change the motion direction or enter 'r' to change modality: ")

        else:
            pub.publish(vel)
            # rospy.loginfo("\nThe imposed velocity is:\n - lin: %f\n - ang: %f ;", vel.linear.x, vel.angular.z)


# function that is called every time that a new message is published on the /check_vel topic
def clbk_velocity(msg):

    """Function that is called every time that a new message is published on the /check_vel topic. 

    If the selected modality is 2, the requested robot velocity is first stored ina global variable and then set. If instead the selected modality is 3, the desired robot 
    velocity is just stored in the global variable, waiting to be checked.

    Args:
        msg (struct): structure containing the velocity requested by the teleop_twist_keyboard module 

    """

    global vel
    global i

    if(mod == "2"):
        vel.linear.x = msg.linear.x
        vel.angular.z = msg.angular.z
        pub.publish(vel)
        # rospy.loginfo("\nThe imposed velocity is:\n - lin: %f\n - ang: %f ;", vel.linear.x, vel.angular.z)
    
    if(mod == "3"):
        vel.linear.x = msg.linear.x
        vel.angular.z = msg.angular.z

    if(mod == "0"):
        if(i == 0):
            i = i+1
        else:
            print('\033[91m' + "\nWarning" + '\033[0m'": choose or change the modality to drive the robot with the keyboard")


# MAIN FUNCTION

def main():

    """Function that first initializes and defines the subscribers and the service, and then simply spins to allow the call-back functions to be called whenever a message arrives on the 
    correspondent communication channel
    """

    rospy.init_node('teleop_mediator_node') # initialize the node with the name 'teleop_mediator_node'

    rospy.Subscriber('/scan', LaserScan, clbk_laser) # initialize and define the subscriber that subscribes to the "/scan" topic and assign the "clbk_laser" call-back function to it 
    rospy.Subscriber('/check_vel', Twist, clbk_velocity) # initialize and define the subscriber that subscribes to the "/check_vel" topic and assign the "clbk_velocity" call-back function to it

    srv = rospy.Service('/change_mod', ChangeMod, clbk_changemod_srv) # initialize and define the server that answers to requests belonging to the "/change_mod" service and assign the "clbk_changemod_srv" call-back function to it
    
    rospy.spin() # spin to allow the call-back functions to be called whenever a message arrives on the correspondent topic or service




if __name__ == '__main__': # if this node is run directly:
    main()