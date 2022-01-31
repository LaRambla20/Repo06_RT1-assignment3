#! /usr/bin/env python

import rospy # import rospy to use ros functionalities
from move_base_msgs.msg import MoveBaseActionGoal # import the type of message that is exchanged on the /move_base/goal topic
from actionlib_msgs.msg import GoalID # import the type of message that is exchanged on the /move_base/cancel topic
from final_assignment_controller.srv import ChangeMod, ChangeModResponse # import both the request message type and the response message type of the ChangeMod.srv custom service

# Publishers
pub1 = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1) # initialize and define the publisher that publishes on the /move_base/goal topic
pub2 = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1) # initialize and define the publisher that publishes on the /move_base/cancel topic

# Client
change_mod = rospy.ServiceProxy('/change_mod', ChangeMod) # initialize and define the client that sends requests belonging to the /change_mod service


# GLOBAL VARIABLES

# Published messages
msg1 = MoveBaseActionGoal()
msg2 = GoalID()


# AUXILIARY FUNCTIONS

# function that is called to set a goal position by publishing on the /move_base/goal topic
def set_goal_position(x, y):
    global msg1
    global pub1

    msg1.goal.target_pose.header.frame_id = "map"
    msg1.goal.target_pose.pose.orientation.w = 1
    msg1.goal.target_pose.pose.position.x = x
    msg1.goal.target_pose.pose.position.y = y
    pub1.publish(msg1)

# function that is called to cancel the previously-set goal position by publishing on the /move_base/cancel topic
def cancel_goal_position():
    global msg2
    global pub2

    msg2.id = ""

    pub2.publish(msg2)

# function that is called to send a request related to the /change_mod service to the server
def switch_to_mod(num):
    global change_mod

    rospy.wait_for_service('/change_mod')
    try:
        res = change_mod(num)
        # print(res)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)


# MAIN FUNCTION (GUI)

def main():

    rospy.init_node("robot_gui_node")
    
    print('\033[93m' + "\n============================================================================\n" + '\033[0m')
    print('\033[93m' + "MODALITIES LEGEND:\n" + '\033[0m')
    print('\033[93m' + " - Modality 1: autonomous navigation to a user-defined target position\n" + '\033[0m')
    print('\033[93m' + " - Modality 2: keyboard-guided navigation\n" + '\033[0m')
    print('\033[93m' + " - Modality 3: keyboard-guided navigation (with assistance in order to\n avoid collisions)\n" + '\033[0m')
    print('\033[93m' + "============================================================================" + '\033[0m')

    while(1):
        mod = input('\033[96m' + "\nEnter here the number of the desired modality, or 'q' to quit the GUI node: " + '\033[0m')

        if(mod == "1"):
            while(1):
                a = input('\033[92m' + "\n# Modality 1 # " + '\033[0m' + "- Enter 'y' to insert a new target, 'r' to change modality: ")

                if(a == "y"):
                    while(1):
                        x = input("Insert new x position: ")
                        try:
                            x = float(x)
                            break
                        except:
                            print("The input was not a number, try again")
                    while(1):
                        y = input("Insert new y position: ")
                        try:
                            y = float(y)
                            break
                        except:
                            print("The input was not a number, try again")

                    set_goal_position(x, y)

                if(a == "r"):
                    cancel_goal_position()
                    break

        if(mod == "2" or mod == "3"):
            while(1):
                switch_to_mod(mod)

                a = input('\033[92m' + "\n# Modality %s # " %mod + '\033[0m' + "- Interact with the GUI in the other terminal window or enter 'r' to change modality: ")

                if(a == "r"):
                    switch_to_mod("0")
                    break
        
        if(mod == "q"):
            break
    
    quit()




if __name__ == '__main__': # if this node is run directly:
    main()