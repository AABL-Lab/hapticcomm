#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import JointState

from hlpr_manipulation_utils.manipulator import Gripper
from armpy import Arm # check this, might be wrong


from kinova_msgs.msg import JointAngles
from kinova_msgs.msg import JointTorque
import numpy as np
from csv import DictWriter, writer

# name the ROS node associated with this program's commands
rospy.init_node("leadermove")

grip = Gripper()
arm = Arm()

# get the joint angles and torques from the arm (initial conditions)
message = rospy.wait_for_message("joint_states", JointState)
message_torque = rospy.wait_for_message("/j2s7s300_driver/out/joint_torques", JointAngles)
print(message.position[5:12])
print(message_torque.joint1)


# When participant is ready, operator selects which card to execute or experiment done
"0) Practice card (triangle)"
"1 or DT - doubletriangle"
"2 or P - pentagon"
"3 ir ST - split triangle"
"4 or J - jetski/half arrow"
"5 or BR - boomerang"
"6 or F - flask/beaker"

# Print the selected card to execute

# Wait for confirmation to start the practice motion, or go back to the menu


# Start recording ROSBAG of the angles and joint efforts


# Start the IMU recording 6DoF data


# Actually move to the points of the trajectory 
# (10 points, one "preview" and one destination for each segment) 
move_robot(self, plan, wait = True)



# Back to the menu for another card choice or finishing.

