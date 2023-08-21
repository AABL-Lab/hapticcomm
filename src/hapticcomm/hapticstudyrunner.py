#!/bin/env python3
#hapticstudylibrary.py
# Kat Allen 2023 
# kat.allen@tufts.edu

from hapticcomm import hapticstudylibrary as hl
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
import time
from study_runner.frames.loggers.rosbag_recorder import RosbagRecorder


def setup_experiment():
# Set up experiment            
    # set experiment number
    print("Experiment Number:")
    try:
        experimentnumber = int(input())
    except:
        print("Experiment Number (integer)")
        experimentnumber = int(input())
        # create folder for IMU and rosbag data for this trial
    
    trialnumber = experimentnumber

    #make a directory for the trial os.makedirs()
    parent_dir = "/home/katallen/catkin_ws/src/hapticcomm/rosbags/"
    newdir = str(trialnumber)
    path = os.path.join(parent_dir, newdir)
    os.makedirs(path)
    print("Directory '%s' created" %trialnumber)

    # set IP address for IMU
    print("Enter IP or just enter for default (10.5.0.6)")
    IPentry = str(input())
    if IPentry =="":
        IP = "10.5.0.6"
    else:
        IP = IPentry
        
    # get order of cards from prerandomized set experiment_card_order.txt
    try:
        filename = "/home/katallen/catkin_ws/src/hapticcomm/experiment_card_order.txt"
        #filename ="experiment_card_order.txt"
        with open(filename, 'r') as f:
            filelist = csv.DictReader(f)    
            cardsorts = [row for row in filelist] #all the card orders
            # just this experiment's cards
            card_dictionary = cardsorts[experimentnumber]
            cards = list(card_dictionary.values()) 
            print("This experiment's cards are, ", cards)
            return cards, IP
    except Exception as error:
        print(error)
#        print("No such file name",filename)


def humanhuman(cards, IP):
    # Start human-human trial        
    # Start upper and side camera
    
    # start IMU, allow participants to begin test card
    print("Ready for test card, triangle, press any key to start IMU")
    flowcontrol = input()
    hl.IMUcontrol("http://"+IP, 1) # start IMU
    
    print("IMU started, begin test card now \n press any key to stop IMU")
    flowcontrol = input()

    hl.IMUcontrol("http://"+IP, 0)    # stop IMU
    
    print("Repeat test card? Y to repeat, q to return to menu, any other key to continue")
    flowcontrol = input()
    if flowcontrol == "q":
        return
    else:
        # start IMU, allow participants to begin card 1
        print("Ready for card 1,", cards[0],"press any key to start IMU")
        startstopIMU = input()
        #start the IMU 
        hl.IMUcontrol("http://"+IP, 1)
        
        print("IMU started, begin card 1 now \n press any key to stop IMU")
        startstopIMU = input()
        hl.IMUcontrol("http://"+IP, 0)
        
        
    
        # stop IMU, trial 1 finished
        
        # start IMU, allow participants to begin card 2
        print("Ready to start card 2 now, ", cards[1])
        startstopIMU = input()
        #start the IMU 
        hl.IMUcontrol("http://"+IP, 1)
        
        print("IMU started, begin card 1 now \n press any key to stop IMU")
        startstopIMU = input()
        # stop IMU, trial 2 finished
        hl.IMUcontrol("http://"+IP, 0)

        
        print("Ready to start card 3 ", cards[2])
        # start IMU, participants begin card 3
        startstopIMU = input()
        #start the IMU 
        hl.IMUcontrol("http://"+IP, 1)
        
        print("IMU started, begin card 3 now \n press any key to stop IMU")
        startstopIMU = input()
        # stop IMU, trial 3 finished
        hl.IMUcontrol("http://"+IP, 0)
        

def robothuman(cards, IP, trialnumber):
    # Introduce robot
    hl.robotspeak("Hello, my name is Boop")

    hl.execute_motion_plan("trajectorypickles/home2start.pkl")
    time.sleep(5)
    runcard("triangle.pkl", IP, trialnumber)

    practice = True
    while practice:
        print("Repeat the practice task?  y to repeat, n to continue, q exit to menu")
        ctrl = input()
        if ctrl =="y":
            practice = True
            print("more practice")
            runcard("triangle.pkl", IP, trialnumber)
        elif ctrl == "n":
            print("done with practice")
            practice = False
        elif ctrl =="q":
            print("returning to menu")
            return
        else:
            print("control input was", ctrl,"try again")
            
    # moving on the new cards
    print("Ready for card 4,", cards[3])
    runcard(cards[3], IP, trialnumber)
    print("Repeat card? y to repeat, q to return to menu, anything else to continue to next card")
    control = input()
    if control == "y": 
        runcard(cards[3], IP, trialnumber)
    elif control =="q":
        print("returning to menu")
        return

    print("Ready for card 5,", cards[4])
    runcard(cards[4], IP, trialnumber)

    print("Ready for last card", cards[5])
    runcard(cards[5], IP, trialnumber)

    print("Enter to open gripper to put down tray")
    input()
    gripper.open()
    hl.robotspeak("Thank you for participating in our study! Goodbye!")
    hl.execute_motion_plan("trajectorypickles/start2home.pkl")
    print("Follower participant complete")
    return
    

def humanleader(cards, IP, trialnumber): 
    # LEADER PARTICIPANT
# Introduce robot
    hl.robotspeak("Hello, my name is Boop")

    hl.execute_motion_plan("trajectorypickles/home2start.pkl")
    time.sleep(5)
    gripper.close()

    
    print("Ready for test card")
    followcard("triangle.pkl", IP, trialnumber)
    practice = True
    while practice:
        print("Repeat the practice task?  y to repeat, n to continue")
        ctrl = input()
        if ctrl =="y":
            practice = True
            print("more practice")
            
            followcard("triangle.pkl", IP, trialnumber)
        elif ctrl == "n":
            practice = False
            print("done with practice")
            
    
    print("Ready for card 4,", cards[3])
    followcard(cards[3], IP, trialnumber)

    print("Ready for card 5,", cards[4])
    followcard(cards[4], IP, trialnumber)

    print("Ready for last card", cards[5])
    followcard(cards[5], IP, trialnumber)

    print("Enter to open gripper to put down tray")
    input()
    gripper.open()
    hl.robotspeak("Thank you for participating in our study! Goodbye!")

def followcard(card, IP, trialnumber):
    startposition = [4.721493795519453,4.448460661610131,-0.016183561810626166,1.5199463284150871,3.0829157579242956,4.517873824894174,0]
    arm.set_velocity(.7)
    print("\n Press enter to move to start position")
    input()
    trajectory = arm.move_to_joint_pose(startposition)
    print("y to open gripper, any other key to continue")
    control = input()
    if control == "y":
        gripper.open()
        print("enter to close gripper")
        input()
        gripper.close()
    hl.robotspeak("I am ready")
    # start cameras

    print("Enter to start the task, ROSbag, and IMU when the participant is ready")
    input()
    # setup ROSBAG every time you make a new file
    # topicslist is a list of strings of topics
    # rosbags is the directory where they will go

    rosbags = "rosbags"
    topicslist = ["/joint_states"]
    recorder = RosbagRecorder(rosbags, topicslist)
    cardprefix, _ = os.path.splitext(card)
    recorder.filename = os.path.join(rosbags, trialnumber, cardprefix)
    # start IMU
    hl.IMUcontrol("http://"+IP, 1)

    print("starting recorder")    
    recorder.start()    
    print("Starting force control mode")
    hl.robotspeak("We can begin")
    arm.start_force_control()


    print("press enter when the participant is done")
    input()
    # stop IMU
    hl.IMUcontrol("http://"+IP, 0)
    # stop ROSBAG
    recorder.stop() # stops the rosbag
    arm.stop_force_control()
    print("Force control stopped, card done")
    
        
def runcard(cardname, IP, trialnumber):
    # start the cameras recording
    startposition = [4.721493795519453,4.448460661610131,-0.016183561810626166,1.5199463284150871,3.0829157579242956,4.517873824894174,0]
    arm.set_velocity(.7)
    print("\n Press enter to move to start position")
    input()
    trajectory = arm.move_to_joint_pose(startposition)
    gripper.open()
    # 
    hl.robotspeak("I am ready")
    print("Enter to start the task when the participant is ready")
    input()
    # setup ROSBAG every time you make a new file
    # topicslist is a list of strings of topics
    # rosbags is the directory where they will go
    rosbags = "rosbags"
    topicslist = ["/joint_states", "/j2s7s300_driver/in/cartesian_force","/j2s7s300_driver/in/cartesian_velocity"]
    recorder = RosbagRecorder(rosbags, topicslist)
    cardprefix, _ = os.path.splitext(cardname)
    recorder.filename = os.path.join(rosbags, trialnumber, cardprefix)
    
    # start IMU
    hl.IMUcontrol("http://"+IP, 1)
    # start recorder
    print("starting the recorder")
    recorder.start()
    print("started the recorder")
    # start trajectory
    print("executing trajectorypickles/"+cardname)
    hl.execute_motion_plan("trajectorypickles/"+cardname)

    hl.IMUcontrol("http://"+IP, 0) # stops the IMU
    recorder.stop() # stops the rosbag

if __name__ == "__main__":
    
    rospy.init_node('hapticcomm')
    arm = armpy.arm.Arm()
    gripper = armpy.gripper.Gripper()
    import rospkg
    rospack = rospkg.RosPack()
    rospack.list()
    hcpath = rospack.get_path('hapticcomm')
    print(hcpath, "is the path used for hapticcomm")
    os.chdir(hcpath)
    defaultcard = ['M.pkl', 'jetski.pkl', 'pentagon.pkl', 'parasail.pkl', 'st.pkl', 'beaker.pkl']
    cards = defaultcard
    IP = "10.5.0.6"
    trialnumber = "test"
    quitcatch = False
    while quitcatch ==False:
        print("\n\n\n\n")	
        print("Haptic Study Runner\n")

        print("1: experiment setup\n 2: human-human\n 3: robot leader \n 4: robot follower \n 0: move to waypoint\n  q to quit")
        menuchoice = input()
        if menuchoice =="q":
            print("exiting")
            quitcatch = True
            exit
        elif menuchoice == "1":
            cards, IP = setup_experiment()

        elif menuchoice == "2":
            if cards == defaultcard:
                print("Default card in use")
                
            humanhuman(cards, IP, trialnumber)
        elif menuchoice == "3":
            if cards == defaultcard:
                print("Default card in use")

            robothuman(cards, IP, trialnumber)
        elif menuchoice == "4":
            if cards == defaultcard:
                print("default card in use")
            humanleader(cards, IP, trialnumber)
        elif menuchoice == "0":
                print("Choose waypoint to move to")
                jointposition = hl.select_waypoint()
                arm.move_to_joint_pose(jointposition)
        else:
            print("Choose again")

            

            
