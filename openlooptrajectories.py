#!/usr/bin/env python

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

outputfile = 'weightlifting_testdata.csv'  # this file should exist and have the headers like this
headersCSV = ['positionname','joint1 angle','joint1 torque','joint2 angle', 'joint2 torque', 'joint3 angle', \
        'joint3 torque', 'joint4 angle', 'joint4 torque','joint5 angle', 'joint5 torque', \
        'joint6 angle', 'joint6 torque', 'joint7 angle', 'joint7 torque']


#arm.move_to_joint_pose(pos)
message = rospy.wait_for_message("joint_states", JointState)
message_torque = rospy.wait_for_message("/j2s7s300_driver/out/joint_torques", JointAngles)
print(message.position[5:12])
print(message_torque.joint1)

triangle1 = {"j2s7s300_joint_1": 2.021561774479218,
             "j2s7s300_joint_2": 1.6345758391089862,
             "j2s7s300_joint_3": 3.3076788307598215,
             "j2s7s300_joint_4": 1.5220344809035852,
             "j2s7s300_joint_5": 0.16646594394147393,
             "j2s7s300_joint_6": 1.6811115945500515,
             "j2s7s300_joint_7": 2.8689121222087977,
             }

triangle2 = {"j2s7s300_joint_1": 1.4585277084073527,
             "j2s7s300_joint_2": 2.687998263037535,
             "j2s7s300_joint_3": 3.2356890591211265,
             "j2s7s300_joint_4": 1.5107799942591436,
             "j2s7s300_joint_5": -0.6428660481725165,
             "j2s7s300_joint_6": 2.55012188603028,
             "j2s7s300_joint_7": 3.6404788921688103
             }

triangle3 = {"j2s7s300_joint_1": 1.3235033106114633,
             "j2s7s300_joint_2": 1.5186257999885713,
             "j2s7s300_joint_3": 3.0703240659742916,
             "j2s7s300_joint_4": 2.459986932620439,
             "j2s7s300_joint_5": -0.6428957424186709,
             "j2s7s300_joint_6": 2.273819455566777,
             "j2s7s300_joint_7": 3.6404714353177585
             }

straightout = {"j2s7s300_joint_1":1.5708, 
        "j2s7s300_joint_2":1.5708, 
        "j2s7s300_joint_3":0, 
        "j2s7s300_joint_4":3.14159,
        "j2s7s300_joint_5":0, 
        "j2s7s300_joint_6":3.14159, 
        "j2s7s300_joint_7":3.14159}

reach45degleftinplane = {"j2s7s300_joint_1":1.5708, "j2s7s300_joint_2":1.5708,
        "j2s7s300_joint_3":4.6807, "j2s7s300_joint_4": 3.8854, 
        "j2s7s300_joint_5":1.4509, "j2s7s300_joint_6":3.1469,
        "j2s7s300_joint_7":3.3695}

reach45degrightinplane = {"j2s7s300_joint_1":1.5863, "j2s7s300_joint_2":1.5002,
        "j2s7s300_joint_3":4.8512, "j2s7s300_joint_4":2.1639, 
        "j2s7s300_joint_5":1.4509, "j2s7s300_joint_6":3.1469, "j2s7s300_joint_7":3.3695}

straightuphigh = {"j2s7s300_joint_1":1.6829, "j2s7s300_joint_2":2.4809, 
        "j2s7s300_joint_3":6.1748, "j2s7s300_joint_4":4.04064, 
        "j2s7s300_joint_5":0.6182, "j2s7s300_joint_6":3.1574, 
        "j2s7s300_joint_7":2.5785}

straightdownlow = {"j2s7s300_joint_1": 1.5642, "j2s7s300_joint_2":0.9189, 
        "j2s7s300_joint_3": 6.1216, "j2s7s300_joint_4":2.4381, 
        "j2s7s300_joint_5":0.6182, "j2s7s300_joint_6":3.2518, 
        "j2s7s300_joint_7":2.5785}

closeholding = {"j2s7s300_joint_1":1.4186, "j2s7s300_joint_2":2.8783,
        "j2s7s300_joint_3":6.5374, "j2s7s300_joint_4":5.6990, 
        "j2s7s300_joint_5":0.05905, "j2s7s300_joint_6":4.4863,
        "j2s7s300_joint_7": 3.0889}

closetoleft =  {"j2s7s300_joint_1":0.89488, "j2s7s300_joint_2":2.5631,
         "j2s7s300_joint_3":5.88381, "j2s7s300_joint_4":5.1607, 
         "j2s7s300_joint_5":-1.0413, "j2s7s300_joint_6":4.6418,
         "j2s7s300_joint_7": 3.7163}

loadingposition = {"j2s7s300_joint_1":2.2842, "j2s7s300_joint_2":1.67622, 
        "j2s7s300_joint_3":7.15522, "j2s7s300_joint_4":5.600, 
        "j2s7s300_joint_5":-0.87966, "j2s7s300_joint_6":4.4367,
        "j2s7s300_joint_7": 2.137}

elbowsout = {"j2s7s300_joint_1":.986, "j2s7s300_joint_2":1.5705, 
        "j2s7s300_joint_3":1.5705, "j2s7s300_joint_4":1.5705, 
        "j2s7s300_joint_5":3.14159, "j2s7s300_joint_6":1.5705,
        "j2s7s300_joint_7": 0}        

print("Do you need to open the gripper to put in or take out an object? y/n")
loadgrip = raw_input()
if loadgrip ==  'y':
    arm.move_to_joint_pose(triangle1)
    
    # make sure the user is ready to catch whatever the gripper is about to drop
    print("open gripper? (get ready to catch!) y/n")
    gripperopen = raw_input()
    if gripperopen == 'y':
        grip.open()
        
    print("Put the weight in the gripper. Ready? y/n")
    ready = raw_input()
    if ready == 'y':
        grip.close()
else: 
    grip.close() # make sure it's closed


print("ready?")
raw_input()



# Move in the demo triangle
arm.move_to_joint_pose(triangle1)
arm.move_to_joint_pose(triangle2)
arm.move_to_joint_pose(triangle3)
arm.move_to_joint_pose(triangle1)

print("Open the gripper? (Get ready to catch the object) y/n")
opengrip = raw_input()
if opengrip == 'y':
    grip.open()


#message = rospy.wait_for_message("joint_states", JointState)
#message_torque = rospy.wait_for_message("/j2s7s300_driver/out/joint_torques", JointAngles)
#torqueandjointarray = np.array(["closeholding", message.position[5], message_torque.joint1, 
#    message.position[6],  message_torque.joint2, message.position[7], 
#    message_torque.joint3, message.position[8], message_torque.joint4,
#    message.position[9], message_torque.joint5, message.position[10], 
#    message_torque.joint6, message.position[11], message_torque.joint7,
#    Fx, Fy, Fz, Tx, Ty, Tz])# write the torque and joint arrays to the CSV file

#with open(outputfile, 'a') as f_object:
#    writer_object = writer(f_object)
#    writer_object.writerow(torqueandjointarray)
#    f_object.close()
#    print("closed the csv")

# bring the arm to a resting position
#arm.move_to_joint_pose(closeholding)

