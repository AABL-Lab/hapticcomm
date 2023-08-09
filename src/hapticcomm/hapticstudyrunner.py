#!/bin/env python3
#hapticstudylibrary.py
# Kat Allen 2023 
# kat.allen@tufts.edu

from hapticcomm import hapticstudylibrary

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
        print("1: gather waypoints\n 2: make trajectory from waypoints\n 3:plan path and move to named waypoint\n 4: load a saved plan \n 5: say something \n g: change gripper status\n free: force control mode \n lock: stop force control mode \nq: exit")
        menuchoice = input()	
        if menuchoice =="q":
            print("exiting")
            quitcatch = True
            exit

        # create folder for IMU and rosbag data for this trial
            
        # Start upper and side camera

        # start IMU, allow participants to begin card 1
        print("Ready for card 1, press any key to start IMU")
        startstopIMU = input()
        #start the IMU 
        
        print("IMU started, begin card 1 now \n press any key to stop IMU")
        startstopIMU = input()
        

        
        # stop IMU, trial 1 finished

        # start IMU, allow participants to begin card 2
        print("IMU started, begin card 2 now")
        
        # stop IMU, trial 2 finished

        # start IMU, participants begin card 3

        # stop IMU, trial 3 finished

            
        # Introduce robot

        # 














        elif menuchoice=="3":
            # plan path to waypoint, by joint position or selected from file
            jointposition = select_waypoint()
            # point is validated, let's go
#            print("Set arm speed 0-1, default is .2")
#            velocity = float(input())
            arm.set_velocity(.5)
            print("\n Moving to", jointposition)
            trajectory = arm.move_to_joint_pose(jointposition)
            print("Arm moved to waypoint", jointposition)
        
        elif menuchoice=="4":
            path = os.getcwd()
            options = os.listdir(path+"/trajectorypickles")
            print(options)
            print("Choose the filename (from trajectorypickles) for the trajectory you want to run")
            planfilename = input()
            
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
