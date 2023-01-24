#!/usr/bin/env python
# 2023 Kat Allen kat.allen@tufts.edu 

import rospy 
from sensor_msgs.msg import JointState
from hlpr_manipulation_utils.manipulator import Gripper
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from kinova_msgs.msg import JointAngles
from kinova_msgs.srv import StartForceControl, StopForceControl
from kinova_msgs.msg import JointTorque
import numpy as np
from csv import DictWriter, writer
import os

def waypointgathering(outputfile='waypoints.csv'):
    # this CSV file should exist with these field names/headers
    # but will be created by the writer if it does not exist

    rospy.init_node("waypointgathering")
    grip = Gripper()
    arm = ArmMoveIt()
    fieldnames=['positionname', "j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7" ]
    start_force_control=rospy.ServiceProxy("/j2s7s300_driver/in/start_force_control", kinova_msgs.srv.StartForceControl)
    stop_force_control=rospy.ServiceProxy("/j2s7s300_driver/in/stop_force_control", kinova_msgs.srv.StopForceControl)
    
    morepoints = True # set up the loop

    while morepoints ==True:
        # get the name of the position
        print("Enter the name of the joint position for the waypoint file, e.g. Triangle 1\n")
        positionname = str(raw_input()) #python2

        # put the robot into kinetic teaching mode so the human can manipulate it
        start_force_control()
        
        print("Kinetic teaching mode engaged. \nMove the arm to the location you want to capture and then press enter\n")

        proceed = str(raw_input()) #python2

        # get the robot out of KT mode
        stop_force_control()

        #get joint configurations at current position
        message = rospy.wait_for_message("joint_states", JointState)

        # create a dictionary of the joint positions by parsing the message from ROS
        joint_vals = {name:val for name,val in zip(message.joint_names, message.position)}

        writer_object = csv.writer(outputfile, delimiter=',')
        # write joints 1-7 from the message to the CSV
        joint_data = [joint_vals[name] for name in fieldnames[1:]]
        writer_object.writerow(positionname, *joint_data)
        writer_object.close()
        print("closed the csv. Waypoint", positionname , "saved\n")

        # check if we should loop
        print("If you are done entering waypoints (to quit the program), press q\n")
        print("To enter more positions/waypoints, press any other key\n")
        loopcheck = raw_input()
        if loopcheck == "q":
            morepoints = False # so the loop will quit
            print("Waypoint Gathering Complete\n")
        else: 
            morepoints = True # loop for more points
