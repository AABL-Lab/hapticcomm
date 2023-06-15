# notes
#https://docs.python-requests.org/en/latest/

#hapticstudyrunner
import hapticstudylibrary as hslibrary
import armpy.arm
import requests

rospy.init_node("hapticstudyrunner")
grip = armpy.gripper.Gripper()
arm = armpy.arm.Arm()
start_force_control=rospy.ServiceProxy("/j2s7s300_driver/in/start_force_control", kinova_msgs.srv.Start)
stop_force_control=rospy.ServiceProxy("/j2s7s300_driver/in/stop_force_control", kinova_msgs.srv.Stop)

# set up the configuration variables for the IMU
imuconfig = {"ipaddr": "10.whatever"}

# set up the configuration variables for ROSBag
rosconfig = {
    "filename":"",
    "log_dir":"",
    "topics":[/tf, /joint_states, /gripper/joint_states, /execute_trajectory/status]
}


def greeting():
    # greet the participants and wave
    hslibrary.robotspeech("Hello, I am Beep. I will be your collaborator today.") 
    hslibrary.robotspeech("I am a robot that can help you with your tray-moving tasks.")
    hslibrary.execute_motion_plan(wave_hello.pkl)

def graspobject(location):
    # move to the object location
    arm.move_robot(location)
    # grasp the object
    grip.open()
    input()
    grip.close()

def demo(trajectorystartlocation, demotrajectoryfilename):
    graspobject(trajectorystartlocation)
    wait(5) # it would be great if this were a verbal command
    hslibrary.execute_motion_plan(demotrajectoryfilename)

class IMUConnect:
    def __init__(self, ipaddr):
        self._ipaddr=ipaddr
    @property # this means we can retrieve ipaddr like a variable    
    def ipaddr(self):# can't be changed from outside the class
        return self._ipaddr 
    def start(self):
        # get the page at the IMU's IP address
        r = requests.get('http://'+str(ipaddr)+'/IMU_on')
        IMU_on
    def stop(self):
        # more stuff from requests

    def __enter__(self):
        self.start()
    def __exit__(self, *args):
        self.stop()
        
        
def robot_leader_sequence()
    print("Robot Leader mode")
    print("Manually start the overhead camera")
    print("press enter when done")
    input()
    cardcounter = 0
    while cardcounter < 3:
        print("Enter the card number to start the motion plan.")
        cardchoice = input()
        if cardchoice == "1":
            trajectoryfilename = "trajectorypickles/doubleM.pkl"
        if cardchoice == "2":
            trajectoryfilename = "trajectorypickles/pentagon.pkl"
        if cardchoice == "3":
            trajectoryfilename = "trajectorypickles/splittriangle.pkl"
        if cardchoice == "4":
            trajectoryfilename = "trajectorypickles/jetski.pkl"
        if cardchoice == "5":
            trajectoryfilename = "trajectorypickles/vee.pkl"
        if cardchoice == "6":
            trajectoryfilename = "trajectorypickles/beaker.pkl"
        else:        
            print("Invalid card choice.",str(cardchoice), "Please try again.")

        # Beep says that they are ready to start
        hslibrary.robotspeech("I am ready to start.")
        # start the trajectory running
        print("Press enter to start the trajectory.")
        input()
    
        print("starting the IMU and ROSBag")
        with IMUConnect(**imuconfig) as imu:    
            # start rosbag recording
            with RosbagRecorder(**rosbagconfig):
                hslibrary.execute_motion_plan(trajectoryfilename)

        print("trajectory complete")
        # stop rosbag recording
        cardcounter = cardcounter + 1
        print("Stop the IMU recording and press enter to continue.")
        input()
    
    # when all 3 cards are done, say goodbye
    hslibrary.robotspeech("Thank you for your help today. Goodbye.")


def robot_follower_sequence():
    print("Robot Follower mode")
    print("Get the IP address of the IMU and open it in a web browser")
    
    print("Manually start the overhead camera")
    print("press enter when done")
    input()
    # Beep says that they are ready to start
    hslibrary.robotspeech("I am ready to start.")
    # start the trajectory running
    print("Press enter to start force-control mode.")
    input()
    # start rosbag recording
                    #FIXME
    start_force_control()
    print("Press enter when the participant is done with the trajectory.")
    input()
    stop_force_control()
    # stop ROSBAG recording
                   # FIXME
    print("Stop the IMU recording and press enter to continue.")    
    input()

print("Welcome to Haptic Study Runner.\n")
print("Choose a mode:\n")
print("1: Full Study sequence\n")
print("RL or 2: Human Follower, Robot Leader\n")
print("RF or 3: Human Leader, Robot Follower\n")
print("T:  Testing Mode\n")
choice = input()

if choice == "1":

 
if choice == "RL" or choice == "rl" or choice == "2":
    robot_leader_sequence()

if choice == "RF" or choice == "rf" or choice == "3":
    robot_follower_sequence()

