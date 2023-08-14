#!/bin/env python3
#hapticstudylibrary.py
# Kat Allen 2023 
# kat.allen@tufts.edu

from hapticcomm import hapticstudylibrary as hl

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
        
        # set IP address for IMU
        print("Enter IP or just enter for default (10.5.0.6)")
        IPentry = str(input())
        if IPentry =="":
            IP = "10.5.0.6"
        else:
            IP = IPentry
            
            # get order of cards from prerandomized set experiment_card_order.txt
        try:
            filename ="experiment_card_order.txt"
            with open(filename, 'r') as f:
                filelist = csv.DictReader(f)    
                cardsorts = [row for row in filelist] #all the card orders
            # just this experiment's cards
            cards = cardsorts(experimentnumber)
        except:
            print("No such file name",filename)
            
def humanhuman():
    # Start human-human trial        
    # Start upper and side camera
    
    # start IMU, allow participants to begin test card
    print("Ready for test card, triangle, press any key to start IMU")
    flowcontrol = input()
    hl.IMUcontrol("http://"+IP, 1)
    
    print("IMU started, begin test card now \n press any key to stop IMU")
    flowcontrol = input()
    
    hl.IMUcontrol("http://"+IP, 0)
    
    print("Repeat test card? Y to repeat, q to return to menu, any other key to continue")
    flowcontrol = input()
    if flowcontrol == "q":
        return
    else:
        # start IMU, allow participants to begin card 1
        print("Ready for card 1,", cards(0),"press any key to start IMU")
        startstopIMU = input()
        #start the IMU 
        hl.IMUcontrol("http://"+IP, 1)
        
        print("IMU started, begin card 1 now \n press any key to stop IMU")
        startstopIMU = input()
        hl.IMUcontrol("http://"+IP, 0)
        
        
    
        # stop IMU, trial 1 finished
        
        # start IMU, allow participants to begin card 2
        print("IMU started, begin card 2 now")
        
        # stop IMU, trial 2 finished
        
        # start IMU, participants begin card 3
        
        # stop IMU, trial 3 finished

def robothuman(cards):
    startposition = [4.721493795519453,4.448460661610131,-0.016183561810626166,1.5199463284150871,3.0829157579242956,4.517873824894174,0]
    arm.set_velocity(.5)
    print("\n Moving to start position")
    trajectory = arm.move_to_joint_pose(startposition)

    # Introduce robot
    hl.robotspeak("Hello, my name is Boop")
    hl.executemotionplan("wave.pkl")

    print("enter to wait 20s, close gripper, wait 5s, and start the demo task")
    input()
    time.sleep(20)
    runcard("triangle.pkl")

    practice = True
    while practice:
        print("Repeat the practice task?  y to repeat, n to continue")
        ctrl = input()
        if ctrl =="y":
            practice == True
            runcard("triangle.pkl")
        elif ctrl == "n":
            practice == False
            
    # moving on the new cards
    print("Ready for card 4,", cards(3))
    runcard(cards(3))

    print("Ready for card 5,", cards(4))
    runcard(cards(4))

    print("Ready for last card", cards(5))
    runcard(cards(5))

    print("Enter to open gripper to put down tray")
    input()
    gripper.open()
    hl.robotspeak("Thank you for participating in our study! Goodbye!")
    print("Follower participant complete\n Enter when ready for leader participant")
    input()
    
# LEADER PARTICIPANT
    startposition = [4.721493795519453,4.448460661610131,-0.016183561810626166,1.5199463284150871,3.0829157579242956,4.517873824894174,0]
    arm.set_velocity(.5)
    print("\n Moving to start position")
    trajectory = arm.move_to_joint_pose(startposition)

    # Introduce robot
    hl.robotspeak("Hello, my name is Boop")
    hl.executemotionplan("wave.pkl")

    print("Ready for test card")
    followcard("triangle")
    practice = True
    while practice:
        print("Repeat the practice task?  y to repeat, n to continue")
        ctrl = input()
        if ctrl =="y":
            practice == True
            
            followcard("triangle.pkl")
        elif ctrl == "n":
            practice == False
    
    print("Ready for card 4,", cards(3))
    followcard(cards(3))

    print("Ready for card 5,", cards(4))
    followcard(cards(4))

    print("Ready for last card", cards(5))
    followcard(cards(5))

    print("Enter to open gripper to put down tray")
    input()
    gripper.open()
    hl.robotspeak("Thank you for participating in our study! Goodbye!")
    print("Follower participant complete\n Enter when ready for leader participant")
    input()

          
    hl.robotspeak("Thank you for participating in our study! Goodbye!")

def followcard(card):
    print("Enger to wait 20 seconds, close gripper, and start force control mode on card", card)
    input()
    gripper.close()
    hl.robotspeak("I am ready")
    # start cameras


    startposition = [4.721493795519453,4.448460661610131,-0.016183561810626166,1.5199463284150871,3.0829157579242956,4.517873824894174,0]
    arm.set_velocity(.5)
    print("\n Press enter to move to start position")
    input()
    trajectory = arm.move_to_joint_pose(startposition)


    print("Enter to start the task when the participant is ready")
    input()
    # start ROSBAG - FIXME
    
    # start IMU
    hl.IMUcontrol("http://"+IP, 1)

    print("Starting force control mode")
    arm.start_force_control()
    # stop IMU
    hl.IMUcontrol("http://"+IP, 0)
    # stop ROSBAG - FIXME

    arm.stop_force_control()
    print("Force control stopped, card done")
    
        
def runcard(cardname):
    # start the cameras recording
    
    # 
    hl.robotspeak("I am ready")
    print("Enter to start the task when the participant is ready")
    input()
    # start ROSBAG - FIXME
    
    # start IMU
    hl.IMUcontrol("http://"+IP, 1)
    # start trajectory
    hl.executemotionplan(cardname)



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
    
    quitcatch = False
    while quitcatch ==False:
        print("\n\n\n\n")	
        print("Haptic Study Runner\n")

        print("1: experiment setup\n 2: human-human\n 3: robot leader \n 4: robot follower \n  q to quit")
        menuchoice = input()
        if menuchoice =="q":
            print("exiting")
            quitcatch = True
            exit
        elif menuchoice == "1":
            


