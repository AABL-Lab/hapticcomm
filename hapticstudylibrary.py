#hapticstudylibrary.py
# Kat Allen 2023 
# kat.allen@tufts.edu

from boto3 import Session
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import os
import sys
import subprocess
from tempfile import gettempdir
import csv
import armpy.arm
import armpy.gripper
import waypointgathering

def robotintroduction(text2speak):
    pass


def create_trajectory_from_waypoints(filename="waypoints.csv"):
    arm = armpy.arm.Arm()

    # filename should be a CSV file, formatted like waypointgathering.py
    print("Loading waypoints from", filename)

    
    with open(filename, 'r') as f:
        filelist = csv.DictReader(f)    
        data = [row for row in filelist]
    
    jointnames = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]    
    print("Moving to start position")

    arm.move_to_joint_pose([float(data[0][joint]) for joint in jointnames])

    print("Select the points to use in the trajectory")
    while point in filelist: 
        point = str(input())
        waypointlist = waypointlist + point

    print("Generating trajectories from waypoints")
    armpy.plan_waypoints(waypointlist)


        
if __name__=="__main__":
    print("\n\n\n\n")	
    print("This is a library file but here are some things to test\n")
    print("1: gather waypoints\n 2: make trajectory from waypoints\n 3:speak\n q: exit")
    menuchoice = input()	
    if menuchoice =="1": 
        print("Gathering waypoints.  Enter filename (or enter to default to waypoints.csv)")
        filename = input()
        if len(filename)==0:
	        waypointgathering.waypointgathering()
        else:
                waypointgathering.waypointgathering(filename)
    elif menuchoice =="2":
        print("making trajectory from waypoints. Enter filename or enter for default (waypoints.csv)")
        filename = input()
        if len(filename)==0:
                create_trajectory_from_waypoints()
        else:        
                create_trajectory_from_waypoints(filename)
    elif menuchoice =="q":
        print("exiting")
        exit
    elif menuchoice=="3":
        print("starting speech. Enter the text to say")
        texttospeak=input()
        robotintroduction(texttospeak)
