#hapticstudyrunner

#Collect data for syncing with IMU: 
# current ROS time
# participant pair ID
# data file to save IMU data into 

#Things to do simultaneously:
    # Start IMU and pass data about time and participants
    # If we are in robot mode, start ROSbag for joint angles and torques
    # If we are in robot-leader mode, select the trajectory, move to the start location
        # and prep to start on confirmation
    # If we are in robot-follower mode, start kinesthetic teaching mode

# Main control page options
    # Mode: 
        # Human-Human 
        # Human-Robot Leader
        # Human-Robot Follower
    # Card order sequence used (all 6 cards should be used, 
    # 3 in H-H and 3 in H-R L and then the same 3 repeated in H-R F)
    # Demo card 
    # introduction and wave
# 