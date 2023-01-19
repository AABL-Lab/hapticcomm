#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import JointState
from hlpr_manipulation_utils.manipulator import Gripper
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from kinova_msgs.msg import JointAngles
from kinova_msgs.msg import JointTorque
import numpy as np
from csv import DictWriter, writer

rospy.init_node("waypointgathering")

grip = Gripper()
arm = ArmMoveIt()

outputfile = 'waypoints.csv'  # this file should exist and have the headers like this
headersCSV = ['positionname','joint1 angle','joint1 torque','joint2 angle', 'joint2 torque', 'joint3 angle', \
        'joint3 torque', 'joint4 angle', 'joint4 torque','joint5 angle', 'joint5 torque', \
        'joint6 angle', 'joint6 torque', 'joint7 angle', 'joint7 torque']

morepoints = True

while morepoints ==True:
    # get the name of the position
    print("Enter the name of the joint position for waypoints.csv, e.g. \"Triangle 1\"")
    positionname = input()

    # put the robot into kinetic teaching mode so the human can manipulate it
    #FIXME how do I get the arm into KT mode?
    print("Kinetic teaching mode engaged. Move the arm to the location you want to capture and then press enter")

    proceed = input()

    # get the robot out of KT mode
    #FIXME how do I get the arm out of KT mode?

    #get joint configurations at current position
    jointposition = {"j2s7s300_joint_1":J1, "j2s7s300_joint_2":J2, "j2s7s300_joint_3":J3, "j2s7s300_joint_4":J4,\
        "j2s7s300_joint_5":J5, "j2s7s300_joint_6":J6, "j2s7s300_joint_7":J7}
    message = rospy.wait_for_message("joint_states", JointState)
    message_torque = rospy.wait_for_message("/j2s7s300_driver/out/joint_torques", JointAngles)
    torqueandjointarray = np.array(["closeholding", message.position[5], message_torque.joint1, 
        message.position[6],  message_torque.joint2, message.position[7], 
        message_torque.joint3, message.position[8], message_torque.joint4,
        message.position[9], message_torque.joint5, message.position[10], 
        message_torque.joint6, message.position[11], message_torque.joint7,
        Fx, Fy, Fz, Tx, Ty, Tz])
        
    # write the torque and joint arrays to the CSV file
    with open(outputfile, 'a') as f_object:
        writer_object = writer(f_object)
        writer_object.writerow(torqueandjointarray)
        f_object.close()
        print("closed the csv. Waypoint", positionname , "saved")

    # check if we should loop
    print("If you are done entering waypoints (to quitthe program), press q")
    print("To enter more positions/waypoints, press any other key")
    loopcheck = input()
    if loopcheck == q:
        morepoints = False # so the loop will quit
    else: 
        morepoints = True # loop for more points