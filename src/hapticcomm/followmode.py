#!/bin/env python3
# followmode.py
# Kat Allen 2023 
# kat.allen@tufts.edu


import armpy.arm
import armpy.gripper
import pickle
# stuff for speech
import rospy
arm = armpy.arm.Arm()

arm.plan_ee_waypoints()

