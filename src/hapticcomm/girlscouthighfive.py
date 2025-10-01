import rospy
import armpy
import os 
import copy

if __name__ == "__main__":
    # these need to be done exactly once across all files
    rospy.init_node('hapticcomm')

    arm = armpy.arm.Arm()

    start_jp = [1.8494435374401599, 2.471101102797667, 3.4542110139420528, 1.228329789577247, -0.17506131300446148, 1.2094786038011212, 3.5375764781770207]
    def start_pose(p):
        p[6] = p[6] + 0.05
        arm.go_to_joint_state(p)

    def high_five_pose(p):
        p[1] = p[1] - 0.4
        # p[2] = p[2] - 0.1
        p[3] = p[3] + 0.4
        p[5] = p[5] - 0.05
        arm.go_to_joint_state(p)

    start_pose(copy.deepcopy(start_jp))
    high_five_pose(copy.deepcopy(start_jp))
    start_pose(copy.deepcopy(start_jp))


