#! /usr/bin/env python

import rospy # to use ros functionalities
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from final_assignment_controller.srv import ChangeMod, ChangeModResponse

# Publisher
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# GLOBAL CONSTANTS

dan_dist = 0.6

# GLOBAL VARIABLES

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
mod = "0"

# Published message
vel = Twist()

# Auxiliary global variables for the GUI aesthetic
i = 0


# SERVICE CALLBACK

def clbk_changemod_srv(req):
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

def clbk_laser(msg):
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


def clbk_velocity(msg):
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

    rospy.init_node('teleop_mediator_node')

    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Subscriber('/check_vel', Twist, clbk_velocity)

    srv = rospy.Service('change_mod', ChangeMod, clbk_changemod_srv)
    
    rospy.spin()




if __name__ == '__main__':
    main()