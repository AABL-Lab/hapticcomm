#!/bin/env python3
#hapticstudylibrary.py
# Kat Allen 2023 
# kat.allen@tufts.edu
try:
    from hapticcomm import waypointgathering
except:
    import waypointgathering
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
import pickle
# stuff for speech
import rospy
import smach
import smach_ros
import actionlib
import hlpr_dialogue_production.msg as dialogue_msgs
import sys
# for controlling the IMU
import requests


def IMUcontrol(url,startstop):
    #requests.get('http://127.0.0.1/foo.php', headers={'host': 'example.com'})
    timeout = 10
    if startstop==1: #start
        print(url+"/IMU_on")
        tryagain = True
        while tryagain:
            try:
                r = requests.get(url+"/IMU_on", headers={'host': 'IMUcontrol.com'}, timeout=timeout)
                print("IMU on requested")
                tryagain = False
            except Exception as error:
                print("error:", error)
                print("try again?")
                control = input()
                if control == "y":
                    tryagain = True
                else:
                    tryagain = False


    elif startstop==0: # stop
        try:
            r = requests.get(url+"/IMU_off", headers={'host': 'IMUcontrol.com'}, timeout=timeout)
            print("IMU off requested")
        except Exception as error:
            print("error:", error)



def getIMUdata(url, filename):
    timeout=60
    try:
        r = requests.get(url+"/preview_last_data", headers={'host': 'IMUcontrol.com'}, timeout=timeout)
        print(type(r),"\n", r)
        # parse the most recent IMU data into a file
        with open(filename, 'a') as f: 
            f.write(response.text) # this does not work
            #            writer_object = csv.writer(f, delimiter=',')
            #            for row in reversed(r):
#                # read from the end until you find the beginning of the last entry
#                while not ("start") in row:
#                    # and write those to the file
#                    writer_object.writerow(row)

        

    except Exception as error:
        print("error:", error)

 
        

def robotspeak(text2speak):
    # modified from test_action_client in hlpr_dialogue_production

    client = actionlib.SimpleActionClient("/HLPR_Dialogue",dialogue_msgs.DialogueActAction)
    rospy.loginfo("waiting for server")
    client.wait_for_server()
    rospy.loginfo("got server")
    s = text2speak
    print("sending", s, "to the speech server")
    client.send_goal(dialogue_msgs.DialogueActGoal(text_or_key=s))
    print("goal sent")
    client.wait_for_result()
    print("got result")
    print(client.get_result())




def predownload_speech():
    robotlexicon = {
                 "greeting": "Hello, My Name Is Boop",
                 "ready": "OK, I am ready to start",
                 "waitgrab":" Wait one moment please while I grab the board",
                 "wait":"Wait one moment, please, I am not yet ready",
                 "goodbye": "Goodbye! It was nice to meet you"
    }
    # downloadable with 
    # AWStext2speech.py in hlpr_dialogue_production. 
    # not sure how to load these back in with it. FIXME


    
def create_trajectory_from_waypoints(filename="waypoints.csv"):
    # filename should be a CSV file, formatted like waypointgathering.py
    print("Loading waypoints from", filename)

    try:
        with open(filename, 'r') as f:
            filelist = csv.DictReader(f)    
            data = [row for row in filelist]
    except:
        print("No such file name", filename)
        return
    jointnames = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]
    # set velocity so for playback - can not be changed at run
    print("Set velocity (0-1, .2 is default)")
    velset = float(input())
    arm.set_velocity(velset)

    print("Select the points to use in the trajectory")
    positionname_list = []
    for row in data:
        positionname_list.append(row['positionname'])
        
    print(positionname_list)
    # loop
    morepoints = True # init
    waypointlist = []    
    while morepoints ==True:
        # get point name
        point = str(input())
        # check if point is in the list data,
        #  [{'positionname': 'beaker5', 'j2s7s300_joint_1': '3.2092276242139905', 'j2s7s300_joint_2': '3.6030092808958316', 'j2s7s300_joint_3': '7.44210992295606', 'j2s7s300_joint_4': '0.8380117736301355', 'j2s7s300_joint_5': '0.0038275429723377183', 'j2s7s300_joint_6': '2.20180744653298', 'j2s7s300_joint_7': '-0.4488199086394775'}]

        for row in data:
            if row['positionname']==point:
                # if it is, add to waypointlist
                print("point found, adding to trajectory")
                goodrow = row.copy() # make a copy disconnected from original
                
                # position name has to be removed from the dictionary
                # of joint positions before it is passed to armpy
                del goodrow['positionname']
                print("row", row, "\n")
                print("goodrow", goodrow, "\n")
                # values need to be converted to floats for armpy
                for k, v in goodrow.items():
                    goodrow[k] = float(v)
                waypointlist.append(goodrow)
                print("New waypoint", goodrow, "added to trajectory \n")
                print("waypoints so far", waypointlist,"\n")
                break # stop looking, we found it
            else:
                print("not in this row")
        print("Get another point? n to generate trajectory,\n any other key to select another point")
        morepointq = input()
        if morepointq=="n":
            morepoints=False
            print("no more points \n")
        else:
            print(positionname_list)
    
    print("Generating trajectories from waypoints:\n")
    print("waypoint list is ", waypointlist, "\n")
    arm.set_velocity(.5)
    trajectory = arm.plan_joint_waypoints(waypointlist)
       
    # now save the trajectory out to a file so we can load it later
    print("name this trajectory/pickle filename")
    trajectoryname = input()
    picklepath = 'trajectorypickles/'+trajectoryname+'.pkl'
    exists = os.path.isfile(picklepath)
    if exists!=True:
        open(picklepath, 'w+').close()
    with open(picklepath, "wb") as f:
        pickle.dump(trajectory, f)

    print("press enter to try out the trajectory")
    input()
    for i, plan in enumerate(trajectory):
        print("executing plan", i) 
        arm.move_robot(plan)

def execute_motion_plan(planfilename="triangle.pkl"):
    arm = armpy.arm.Arm()
    with open (planfilename, "rb") as f:
        plan = pickle.load(f)
        
    print("running the loaded trajectory", planfilename)
    print("Executing the trajectory")
    for i, plan in enumerate(plan):
        print("executing plan", i) 
        arm.move_robot(plan)


def select_waypoint():
    print("Enter filename for waypoint from file, 0 to specify joints manually, or enter to select from waypoints.csv")
    filenameselect = input()
    if filenameselect=="0":
        goodposition=False
        print("Getting joint positions manually")
        while goodposition== False:
            print("Enter the joint positions 1-7, separated by commas")
            print("ex: 3.5, 2, -1, 0, 4, 0, 1.5")
            jointpositions=input()
            jointposition = [float(value) for value in jointpositions.split(",")]
            
            
            print("Planning to joint position",jointposition, "enter to continue or n to enter again")
            userentry = input()
            if userentry =="n":
                goodposition=False
            else:
                goodposition=True

    elif filenameselect=="":
        filename = "waypoints.csv"
    else:
        filename = filenameselect    

    if filenameselect !="0":
        print("Loading waypoints from", filename)
        with open(filename, 'r') as f:
            filelist = csv.DictReader(f)    
            data = [row for row in filelist]
    
            jointnames = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]    

            print("Select the points to use in the trajectory")
            positionname_list = []
            for row in data:
                positionname_list.append(row['positionname'])
            print("\n -----------------------------------\n" ,positionname_list)
        # loop
        point_validated = False
        while point_validated==False:
    # Get the point name entered by the user
            point = str(input())
    # validate that the point they entered is in the list of waypoints
            for row in data:
                if row['positionname']==point:
                    print("point found")
                    goodrow = row.copy() # make a copy disconnected from original
                    # position name has to be removed from the dictionary
                    # of joint positions before it is passed to armpy
                    del goodrow['positionname']
                    # values need to be converted to floats for armpy
                    for k, v in goodrow.items():
                        goodrow[k] = float(v)
                    print("Waypoint", goodrow, "selected")
                    print("Ready to move to waypoint", goodrow)
                    print("\n Press y to move to waypoint", goodrow, "or any other key to select again")
                    goodwaypoint = str(input())
                    if goodwaypoint=="y":
                        point_validated=True
                        jointposition = goodrow
                        break 
                    else:
                        "Selecting another waypoint"
                else: 
                    print("Enter the point you want to navigate to")
                    print(positionname_list)
    return jointposition
        


                        
if __name__=="__main__":
    # these need to be done exactly once across all files
    rospy.init_node('hapticcomm')
    arm = armpy.arm.Arm()
    gripper = armpy.gripper.Gripper()
    import rospkg
    rospack = rospkg.RosPack()
    rospack.list()
    hcpath = rospack.get_path('hapticcomm')
    print(hcpath, "is the path used for hapticcomm")
    os.chdir(hcpath)
    
    quitcatch = False
    while quitcatch ==False:
        print("\n\n\n\n")	
        print("This is a library file but here are some things to test\n")
        print("1: gather waypoints\n 2: make trajectory from waypoints\n 3:plan path and move to named waypoint\n 4: load a saved plan \n 5: say something \n g: change gripper status\n free: force control mode \n lock: stop force control mode \n IMU: start and stop recording \n speed: set arm motion speed \nq: exit")
        menuchoice = input()	
        if menuchoice =="1": 
            print("Gathering waypoints.  Enter filename (or enter to default to waypoints.csv)")
            filename = input()
            if len(filename)==0:
                waypointgathering.gather_waypoints()
            else:
                waypointgathering.gather_waypoints(filename)
        elif menuchoice =="2":
            print("making trajectory from waypoints. Enter filename or enter for default (waypoints.csv)")
            filename = input()
            if len(filename)==0:
                create_trajectory_from_waypoints()
            else:        
                create_trajectory_from_waypoints(filename)
        elif menuchoice =="q":
            print("exiting")
            quitcatch = True
            exit
        elif menuchoice=="3":
            jointposition = select_waypoint()
            # point is validated, let's go
#            print("Set arm speed 0-1, default is .2")
#            velocity = float(input())
#            arm.set_velocity(.5)
            print("\n Moving to", jointposition)
            trajectory = arm.move_to_joint_pose(jointposition)
            print("Arm moved to waypoint", jointposition)
        
        elif menuchoice=="4":
            path = os.getcwd()
            options = os.listdir(path+"/trajectorypickles")
            print(options)
            print("Choose the filename (from trajectorypickles) for the trajectory you want to run")
            planfilename = input()
            # velocity is set when you create the pickle
            execute_motion_plan("trajectorypickles/"+planfilename)

            
        elif menuchoice=="5":
            print("Text to speak?\n")
            speaktext = input()
            robotspeak(speaktext)

        elif menuchoice == "g":
            print("Open (o) or close (c) or back (any other key)?")
            grippercommand = input()
            if grippercommand == "o":
                gripper.open()
            elif grippercommand == "c":
                gripper.close()
            else:
                pass

        elif menuchoice == "free":
            print("Starting force control mode")
            arm.start_force_control()
            
        elif menuchoice == "lock":
            print("Stop force control mode")
            arm.stop_force_control()

        elif menuchoice =="IMU":
            print("1: start, 0: stop, anything else: go back")
            control = input()
            URL = "http://10.5.0.6"
            if control == "0":
                print("sending stop to", URL)
                IMUcontrol(URL ,0)

            elif control == "1":
                print("Sending start to", URL)
                IMUcontrol(URL, 1)
            else:
                pass
        elif menuchoice =="speed":
            print("Set the arm speed, 0-1")
            speed=float(input())
            arm.set_velocity(speed)
        elif menuchoice =="tm":
            robotspeak("Hello, my name is Boop")
            # move at the same time
        else:
            pass
                

        
