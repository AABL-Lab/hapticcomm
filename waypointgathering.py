#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import JointState
from hlpr_manipulation_utils.manipulator import Gripper
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from kinova_msgs.msg import JointAngles
from kinova_msgs.msg import JointTorque
import numpy as np
from csv import DictWriter, writer
import os

rospy.init_node("waypointgathering")

grip = Gripper()
arm = ArmMoveIt()

outputfile = 'waypoints.csv'  # this file should exist and have the headers like this
headersCSV = ['positionname','joint1 angle','joint2 angle', 'joint3 angle', \
        'joint4 angle', 'joint5 angle', \
              'joint6 angle', 'joint7 angle']

morepoints = True

while morepoints ==True:
    # get the name of the position
    print("Enter the name of the joint position for waypoints.csv, e.g. \"Triangle 1\"")
    positionname = str(raw_input())

    # put the robot into kinetic teaching mode so the human can manipulate it
    os.system("rosservice call /j2s7s300_driver/in/start_force_control")

    print("Kinetic teaching mode engaged. Move the arm to the location you want to capture and then press enter")

    proceed = str(raw_input())

    # get the robot out of KT mode
    os.system("rosservice call /j2s7s300_driver/in/stop_force_control")

    #get joint configurations at current position
    message = rospy.wait_for_message("joint_states", JointState)
    jointposition = {"positionname": positionname, "j2s7s300_joint_1":message.position[5], "j2s7s300_joint_2":message.position[6], "j2s7s300_joint_3":message.position[7], "j2s7s300_joint_4":message.position[8],"j2s7s300_joint_5":message.position[9], "j2s7s300_joint_6":message.position[10], "j2s7s300_joint_7":message.position[11]}
        writer_object = writer(f_object)
        writer_object.writerow([jointposition])
        f_object.close()
        print("closed the csv. Waypoint", positionname , "saved")

    # check if we should loop
    print("If you are done entering waypoints (to quitthe program), press q")
    print("To enter more positions/waypoints, press any other key")
    loopcheck = raw_input()
    if loopcheck == "q":
        morepoints = False # so the loop will quit
    else: 
        morepoints = True # loop for more points
