#hapticstudyrunner
import hapticstudylibrary as hslibrary

'''
Human-Human mode:
- consent form on qualtrics
- read study instructions
- let participants test out triangle card
'''




'''
for each card: 1-3
- start overhead camera
- synchronize IMU time (optional)
- start IMU recording
- do card
- stop IMU recording
- stop overhead camera
- have participants do qualtrics survey question

Human-Robot Follower Mode
- start overhead camera
- synchronize IMU time
- put robot in k-t mode
                        - make robot talk
'''

robotspeak(speaktext)

'''
  - start IMU recording
- start ROSBAG recording
- put robot in K-T mode
                        - start any visual tracking from robot camera
- do card
- stop IMU recording
- stop ROSBAG recording
- stop kinesthetic-teaching mode
- stop overhead camera

Human-Robot Leader Mode
- start overhead camera
- synchronize IMU time
                        - make robot talk
                        - turn on any visual tracking from robot camera
- start IMU recording
- start ROSBAG recording
- start robot playing selected trajectory/card
- stop IMU recording
- stop ROSBAG recording
                        - stop any visual tracking from robot camera
- stop overhead camera

'''


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
    # Card order sequence used (all 6 cards should be used) 
    # 3 in H-H and 3 in H-R L and then the same 3 repeated in H-R F)
    # Run demo card/triangle
         # in H-H mode
         # in H-R leader mode
         # in H-R follower mode
    # robot introduction and wave


print("Welcome to Haptic Study Runner.\n")
print("Choose a mode:\n")
print("H: Human-Human\n")
print("RF: Human Leader, Robot Follower\n")
print("RL: Human Follower, Robot Leader\n")
print("T:  Testing Mode\n")
choice = input()

if choice =="H":
    IMUdefaultIP = 0.0.
    print("Enter the IMU IP address. Default is ", IMUdefaultIP)
    print("Ready to start the IMU at", IMUIP)
    hslibrary.startIMU(IMUIP)
