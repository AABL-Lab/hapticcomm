import sys
import os
import csv
import rosbag
import rospy

##################
# DESCRIPTION:
# Creates CSV files of the robot joint states from a rosbag (for visualization with e.g. pybullet)
# 
# USAGE EXAMPLE:
# rosrun your_package get_jstate_csvs.py /root/catkin_ws/bagfiles your_bagfile.bag
# ##################

filename = sys.argv[2]
directory = sys.argv[1]
print("Reading the rosbag file")
if not directory.endswith("/"):
  directory += "/"
extension = ""
if not filename.endswith(".bag"):
  extension = ".bag"
bag = rosbag.Bag(directory + filename + extension)

# Create directory with name filename (without extension)
results_dir = directory + filename[:-4] + "_results"
if not os.path.exists(results_dir):
  os.makedirs(results_dir)

print("Writing robot joint state data to CSV")
force = [0,0,0,0,0,0]
with open(results_dir +"/"+filename+'_forcetorque-calibrated.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['time', 'forcex', 'forcey','forcez','torquex','torquey','torquez'])
  # Get all message on the /ft_sensor/ft_compensated topic
  for topic, msg, t in bag.read_messages(topics=['/ft_sensor/ft_compensated']):
      t_epoch = msg.header.stamp.sec
      t = time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime(t_epoch))
      force[0]=msg.wrench.force.x
      force[1]=msg.wrench.force.y
      force[2]=msg.wrench.force.z
      force[3]=msg.wrench.torque.x
      force[4]=msg.wrench.torque.y
      force[5]=msg.wrench.torque.z
      data_writer.writerow([t, force[0], force[1], force[2], force[3], force[4], force[5], force[6]])

print("Finished creating csv file!")
bag.close()
