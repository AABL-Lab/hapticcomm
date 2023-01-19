#!/usr/bin/env python
# This is written for Python2, not updated for Python3
# known items to translate marked with #Python2

import rospy 
from sensor_msgs.msg import JointState
from hlpr_manipulation_utils.manipulator import Gripper
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from kinova_msgs.msg import JointAngles
from kinova_msgs.msg import JointTorque
import numpy as np
from csv import DictWriter, writer

rospy.init_node("weightlifting")

grip = Gripper()
arm = ArmMoveIt()

outputfile = 'waypoints.csv'  # this file should exist and have the headers like this
headersCSV = ['positionname','joint1 angle','joint1 torque','joint2 angle', 'joint2 torque', 'joint3 angle', \
        'joint3 torque', 'joint4 angle', 'joint4 torque','joint5 angle', 'joint5 torque', \
        'joint6 angle', 'joint6 torque', 'joint7 angle', 'joint7 torque']

import trajectories # all the trajectories we will need, pre-calculated from waypoints.py
import waypoints # in case we need to jump back to any of the waypoints


# User menu to choose a card or set follower mode
# User menu to choose a card or set follower mode
while True:
    print("press  \n \
        0: Demo Triangle card\n \
        1: Card 1\n\
        2: Card 2\n \
        3: Card 3\n \
        4: Card 4\n \
        5: Card 5\n \
        6: Card 6\n \
        7: Move to starting position to load gripper\n \
        8: Follower mode \n \
        q: Quit")

    choice = input()
    if choice == "0":  
        print("Starting the Demo Card. Confirm (c) or back (b)")
        confirm = input()
        if confirm =="c" or confirm =="": 
                demotrianglecard()
        else: 
                pass
 
    elif choice == "1": 
            print("Starting Card 1. Confirm (c or enter) or back (b)")
            confirm = input()
            if confirm =="c" or confirm =="": 
                demotrianglecard()
            else: 
                pass

            card1()
    elif choice == "2":
        print("Starting Card 2")
        confirm = input()
        if confirm =="c" or confirm =="": 
            demotrianglecard()
        else: 
            pass

# 




def loadgrip(): 
    arm.move_to_joint_pose(waypoints.start) 
    # make sure the user is ready to catch whatever the gripper is about to drop
    print("open gripper? (get ready to catch!) y/n")
    gripperopen = raw_input() #Python2
    if gripperopen == 'y':
        grip.open()
        
    print("Put the weight in the gripper. Ready? y/n")
    ready = raw_input() #Python2
    if ready == 'y':
        grip.close()


# Move in the demo triangle
arm.move_to_joint_pose(triangle1)
arm.move_to_joint_pose(triangle2)
arm.move_to_joint_pose(triangle3)
arm.move_to_joint_pose(triangle1)



print("Open the gripper? (Get ready to catch the object) y/n")
opengrip = raw_input()
if opengrip == 'y':
    grip.open()



