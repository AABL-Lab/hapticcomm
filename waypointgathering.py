#!/usr/bin/env python
# 2023 Kat Allen kat.allen@tufts.edu 

import rospy 
from sensor_msgs.msg import JointState
from kinova_msgs.msg import JointAngles
from kinova_msgs.msg import JointTorque
import numpy as np
import csv 
import armpy.arm
import armpy.gripper

def waypointgathering(outputfile='waypoints.csv'):
    # this CSV file should exist with these field names/headers
    # but will be created by the writer if it does not exist

#    rospy.init_node("waypointgathering")
    fieldnames=['positionname', "j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7" ]
    start_force_control=arm.start_force_control
    stop_force_control= arm.stop_force_control
    
    morepoints = True # set up the loop
    all_points=[]

    while morepoints ==True:
        # get the name of the position
        print("Enter the name of the joint position for the waypoint file, e.g. Triangle 1\n")
        positionname = input()

        # put the robot into kinetic teaching mode so the human can manipulate it
        start_force_control()
        
        print("Kinetic teaching mode engaged. \nMove the arm to the location you want to capture and then press enter\n")

        proceed = str(input())

        # get the robot out of KT mode
        stop_force_control()

        #get joint configurations at current position
        message = rospy.wait_for_message("joint_states", JointState)

        # create a dictionary of the joint positions by parsing the message from ROS
        joint_vals = {name:val for name,val in zip(message.name, message.position)}

        joint_data = [joint_vals[name] for name in fieldnames[1:]]
        all_points.append([positionname, *joint_data])

        print("closed the csv. Waypoint", positionname , "saved\n")

        # check if we should loop
        print("If you are done entering waypoints (to quit the program), press q\n")
        print("To enter more positions/waypoints, press any other key\n")
        loopcheck = input()
        if loopcheck == "q":
            morepoints = False # so the loop will quit
            print("Waypoint Gathering Complete\n")
        else: 
            morepoints = True # loop for more points

    with open(outputfile, 'a') as f: 
        writer_object = csv.writer(f, delimiter=',')
        writer_object.writerow(fieldnames)
        # write joints 1-7 from the message to the CSV, for all of the points
        for points in all_points:	
            writer_object.writerow(points)



def waypoints2trajectories(waypointsfile ="waypoints.csv"):
    fieldnames=['positionname', "j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7" ]
    # load in the waypoints file as a dictionary with position name as a key and a list of the joint positions
    waypoints = {}
    with open(waypointsfile, newline='\n') as file:
        reader_object =csv.DictReader(file, delimiter=',')
        rownum=0
        for row in reader_object:
            rownum += 1
            #print("row", rownum, row)
            # use the position name as the key for a dictionary entry where the value is a list of joint positions
            waypoints[row["positionname"]]= [row["j2s7s300_joint_1"], row["j2s7s300_joint_2"], row["j2s7s300_joint_3"], \
                        row["j2s7s300_joint_4"],row["j2s7s300_joint_5"],row["j2s7s300_joint_6"],row["j2s7s300_joint_7"]]

    # now present those points to the user to select for inclusion in the trajectory
    print(waypoints.keys())
    print("Select waypoints by name, in order, for inclusion in the trajectory.\n")
    print("Enter an empty line when finished")
    morepoints=True # init
    pointnames = []
    while morepoints==True:
        input_pointname = input()
        if input_pointname=="":
            morepoints=False

            print("points complete")
            print(*pointnames)
        else:
            if input_pointname in waypoints.keys(): # check that it's a real waypoint, spelling is correct, etc
                pointnames.append(input_pointname) # this should make a list of the point names as keys into the waypoints dict
                print(input_pointname, "added")
            else:
                print("That waypoint does not exist in the csv, check your spelling and re-enter")
    if pointnames == []:
        print("There are no points in the list to generate a trajectory")
    else: 
        print("Generating a trajectory based on entered points")
        trajectory = armpy.plan_joint_waypoints(self, pointnames, starting_config=None)
    
    return trajectory
    # what should I do with the trajectory to save it?

if __name__=="__main__":
# might need this 
	#armpy.move_to_point(jointpositionlist) # move to the joint position, defined as a 7dof list
    rospy.init_node("waypointgathering")
    gripper = armpy.gripper.Gripper()
    arm = armpy.arm.Arm()

    print("Gather trajectories using hapticstudylibrary.py")
    #waypoints2trajectories("waypoints.csv")
