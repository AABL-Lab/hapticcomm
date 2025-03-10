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
import forcetorquecontrol.forcetorquecontrol as ft

startposition = [1.5920214543667193,1.866893900482904,2.802151733686583,1.527687738229784,6.1766950825672415,1.807115121916386,-1.1973379181817223]

def setup_experiment():
# Set up experiment            
    # set experiment number
    experimentnumber=""
    got_experimentnumber=False
    while not got_experimentnumber:
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
        try:
            newdir = str(trialnumber)
            path = os.path.join(parent_dir, newdir)
            os.makedirs(path)
            print("Directory '%s' created" %trialnumber)
            got_experimentnumber=True
        except FileExistsError:
            print("Directory '%s' already exists" %trialnumber)


    # set IP address for IMU
    print("Enter IP or just enter for default (10.5.13.115)")
    IPentry = str(input())
    if IPentry =="":
        IP = "10.5.13.115"
    else:
        IP = IPentry
        
    # get order of cards from prerandomized set experiment_card_order.txt
    try:
        filename = "/home/katallen/catkin_ws/src/hapticcomm/experiment_card_order.txt"
        cardset = experimentnumber % 6
        print("cardset is ", cardset)
        print("Using Card Set", cardset)
        with open(filename, 'r') as f:
            filelist = csv.DictReader(f)    
            cardsorts = [row for row in filelist] #all the card orders
            # just this experiment's cards
            card_dictionary = cardsorts[cardset-1]
            cards = list(card_dictionary.values()) 
            print("\n\n\nThis experiment's cards are, ", cards)
            return trialnumber, cards, IP
    except Exception as e: 
        print("cards not set, ", e)
        exit


    
def humanhuman(cards, IP, trialnumber):
    # Start human-human trial        
    # Start upper and side camera
    testcard = True
    while testcard == True:
        # start IMU, allow participants to begin test card
        print("Ready for test card, triangle, press any key to start IMU")
        flowcontrol = input()
        hl.IMUcontrol("http://"+IP, 1) # start IMU
        
        print("IMU started, begin test card now \n press any key to stop IMU")
        flowcontrol = input()
        
        hl.IMUcontrol("http://"+IP, 0)    # stop IMU
        
        print("Repeat test card? Y or y to repeat, q to return to menu, any other key to continue")
        flowcontrol = input()
        if flowcontrol == "q":
            return
        elif flowcontrol =="Y" or flowcontrol =="y":
            testcard = True
        else:
            testcard = False

    card1 = True
    while card1:
        # start IMU, allow participants to begin card 1
        print("Ready for card 1,", cards[0],"press any key to start IMU")
        startstopIMU = input()
        #start the IMU 
        hl.IMUcontrol("http://"+IP, 1)
        
        print("IMU started, begin card 1 now \n press any key to stop IMU")
        startstopIMU = input()
        hl.IMUcontrol("http://"+IP, 0)
        print("Repeat card 1? Y to repeat, q to quit, any other key to continue to card 2")
        flowcontrol = input()
        if flowcontrol == "q":
            return
        elif flowcontrol =="Y" or flowcontrol =="y":
            card1 = True
        else:
            card1 = False
    
    # stop IMU, trial 1 finished
    print("\n\n\n\nTell the participants to do one question of the survey now\n\n\n press enter when they are done")
    control = input()

    # start IMU, allow participants to begin card 2
    card2 = True
    while card2:
        print("Ready to start card 2 now, ", cards[1])
        startstopIMU = input()
        #start the IMU 
        hl.IMUcontrol("http://"+IP, 1)
        
        print("IMU started, begin card 2 now \n press any key to stop IMU")
        startstopIMU = input()
        # stop IMU, trial 2 finished
        hl.IMUcontrol("http://"+IP, 0)

        print("Repeat card 2? Y to repeat, q to quit, any other key to continue to card 3")
        flowcontrol = input()
        if flowcontrol == "q":
            return
        elif flowcontrol =="Y" or flowcontrol =="y":
            card2 = True
        else:
            card2 = False
        
    print("\n\n\n\nTell the participants to do one question of the survey now\n\n\n press enter when they are done")
    control = input()

    card3 = True
    while card3:
        print("Ready to start card 3 ", cards[2])
        # start IMU, participants begin card 3
        startstopIMU = input()
        #start the IMU 
        hl.IMUcontrol("http://"+IP, 1)
        
        print("IMU started, begin card 3 now \n press any key to stop IMU")
        startstopIMU = input()
        # stop IMU, trial 3 finished
        hl.IMUcontrol("http://"+IP, 0)
        
        print("Repeat card 3? Y to repeat, q to quit, any other key to finish")
        flowcontrol = input()
        if flowcontrol == "q":
            return
        elif flowcontrol =="Y" or flowcontrol =="y":
            card3 = True
        else:
            card3 = False

    print("\n\n\n\nTell the participants to do one question of the survey now\n\n\n press enter when they are done")
    control = input()
    print("Done with human-human trials. Download data from IMU")
    return    

def robotleader(cards, IP, trialnumber):
    # Introduce robot
    print("Enter to have Beep introduce and move to start position")
    input()
    hl.robotspeak("Hello, my name is Beep")
    hl.execute_motion_plan("trajectorypickles/home2start.pkl")
    print("press enter to start the practice task")
    step = input()
    print("trialnumber",trialnumber)
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
    print("\n\n\nReady for card,", cards[3])
    runcard(cards[3], IP, trialnumber)
    print("Repeat card? y to repeat, q to return to menu, anything else to continue to next card \n Don't forget to have them do the survey question!")
    control = input()
    if control == "y": 
        runcard(cards[3], IP, trialnumber)
    elif control =="q":
        print("returning to menu")
        return

    print("\n\n\nReady for card,", cards[4])
    runcard(cards[4], IP, trialnumber)
    print("Repeat card? y to repeat, q to return to menu, anything else to continue to next card \n Don't forget to have them do the survey question!")
    control = input()
    if control == "y": 
        runcard(cards[4], IP, trialnumber)
    elif control =="q":
        print("returning to menu")
        return


    print("\n\n\nReady for last card", cards[5])
    runcard(cards[5], IP, trialnumber)
    print("Repeat card? y to repeat, q to return to menu, anything else to continue to next card \n Don't forget to have them do the survey question!")
    control = input()
    if control == "y": 
        runcard(cards[5], IP, trialnumber)
    elif control =="q":
        print("returning to menu")
        return


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
    print("press enter to have Beep introduce themselves")
    input()
    hl.robotspeak("Hello, my name is Beep")
    hl.execute_motion_plan("trajectorypickles/home2start.pkl")
    time.sleep(5)
    print("press enter to open the gripper")
    input()
    gripper.open()
    
    print("\n\n\n Ready for test card")
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
            
    
    print("\n\n\nReady for card 4,", cards[3])
    followcard(cards[3], IP, trialnumber)
    print("Repeat card? y to repeat, q to return to menu, anything else to continue to next card \n Don't forget to have them do the survey question!")
    control = input()
    if control == "y": 
        followcard(cards[3], IP, trialnumber)
    elif control =="q":
        print("returning to menu")
        return


    print("\n\n\n Ready for card 5,", cards[4])
    followcard(cards[4], IP, trialnumber)
    print("Repeat card? y to repeat, q to return to menu, anything else to continue to next card \n Don't forget to have them do the survey question!")
    control = input()
    if control == "y": 
        followcard(cards[4], IP, trialnumber)
    elif control =="q":
        print("returning to menu")
        return


    print("\n\n\n Ready for last card", cards[5])
    followcard(cards[5], IP, trialnumber)
    print("Repeat card? y to repeat, q to return to menu, anything else to continue to next card\n Don't forget to have them do the survey question!")
    control = input()
    if control == "y": 
        followcard(cards[5], IP, trialnumber)
    elif control =="q":
        print("returning to menu")
        return


    print("Enter to open gripper to put down tray")
    input()
    gripper.open()
    hl.robotspeak("Thank you for participating in our study! Goodbye!")
    arm.set_velocity(.5)
    arm.move_to_joint_post(startposition)
    hl.execute_motion_plan("trajectorypickles/start2home.pkl")
    print("Leader participant complete")
    
def followcard(card, IP, trialnumber):    
    print("Enter to go to start position. Make sure the arm is clear and gripper empty!")
    input()
    arm.set_velocity(.5)
    arm.move_to_joint_pose(startposition)

    print("press enter to close the gripper")
    input()
    gripper.close()
    hl.robotspeak("I am ready")
    # start cameras

    print("Check that the cameras are on \n Press enter to start the task, ROSbag, and IMU when the participant is ready")
    input()
    # setup ROSBAG every time you make a new file
    # topicslist is a list of strings of topics
    # rosbags is the directory where they will go

    rosbags = "rosbags"
    topicslist = ["/joint_states", "/j2s7s300_driver/in/cartesian_force","/j2s7s300_driver/in/cartesian_velocity", "/ft_sensor/ft_compensated", "/bota/ft_sensor0/ft_sensor_readings/wrench", "/bota/ft_sensor0/ft_sensor_readings/imu"]

    recorder = RosbagRecorder(rosbags, topicslist)
    cardprefix, _ = os.path.splitext(card)
    recorder.filename = os.path.join(rosbags, str(trialnumber), cardprefix)
    # start IMU
    hl.IMUcontrol("http://"+IP, 1)

    print("starting recorder")    
    recorder.start()    
    print("Starting forcetorquecontrol mode")
    hl.robotspeak("We can begin")

    ft_controller.start()

    print("press enter when the participant is done")
    input()
    ft_controller.stop()
    # stop IMU
    hl.IMUcontrol("http://"+IP, 0)
    # stop ROSBAG
    recorder.stop() # stops the rosbag

    print("Forcetorquecontrol stopped, card done. Press enter to open the gripper")
    input()
    gripper.open()
    
        
def runcard(cardname, IP, trialnumber):
    hl.robotspeak("I am ready")
    print("enter to close gripper")
    input()
    gripper.close()
    print("Enter to start the task when the participant is ready")
    input()
    # setup ROSBAG every time you make a new file
    # topicslist is a list of strings of topics
    # rosbags is the directory where they will go
    rosbags = "rosbags"
    topicslist = ["/joint_states", "/j2s7s300_driver/out/cartesian_command","/j2s7s300_driver/in/cartesian_velocity", "/ft_sensor/ft_compensated", "/bota/ft_sensor0/ft_sensor_readings/wrench", "/bota/ft_sensor0/ft_sensor_readings/imu"]
    recorder = RosbagRecorder(rosbags, topicslist)
    cardprefix, _ = os.path.splitext(cardname)
    recorder.filename = os.path.join(rosbags, str(trialnumber), cardprefix)
    
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
    print("Enter to open the gripper")
    input()
    gripper.open() 


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
    IP = "10.5.13.115"
    trialnumber = "test"

    ft_controller = ft.ForceTorqueController(controltype="PD",
                                               K_P=-0.4, K_D=-5.0, threshold=2.0)
    print("Force torque controller follow mode ready")


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
            trialnumber, cards, IP = setup_experiment()

        elif menuchoice == "2":
            if cards == defaultcard:
                print("Default card in use")
                
            humanhuman(cards, IP, trialnumber)
        elif menuchoice == "3":
            if cards == defaultcard:
                print("Default card in use")

            robotleader(cards, IP, trialnumber)
        elif menuchoice == "4":
            if cards == defaultcard:
                print("default card in use")
            humanleader(cards, IP, trialnumber)
        elif menuchoice == "0":
                print("Choose waypoint to move to")
                jointposition = hl.select_waypoint()
                arm.set_velocity(.5)
                arm.move_to_joint_pose(jointposition)
        else:
            print("Choose again")

            

            
