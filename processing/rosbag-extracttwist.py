import sys
import os
import csv
import rosbag
import rospy
import datetime
import time
import glob
##################
# DESCRIPTION:
# Creates CSV files of the robot joint states from a rosbag (for visualization with e.g. pybullet)
# 
# USAGE EXAMPLE:
# rosrun your_package get_jstate_csvs.py /root/catkin_ws/bagfiles your_bagfile.bag
# ##################

#filename = sys.argv[2]
directory = sys.argv[1]

def get_files(path, extension, recursive=False):
    """
    A generator of filepaths for each file into path with the target extension.
    If recursive, it will loop over subfolders as well.
    """
    if not recursive:
        for file_path in glob.iglob(path + "/*." + extension):
            yield file_path
    else:
        for root, dirs, files in os.walk(path):
            for file_path in glob.iglob(root + "/*." + extension):
                print(file_path)
                res = file_path.split('/')
                global filename
                global subdir
                filename = res[-1]
                subdir = res[-2]
                print(filename)
                yield file_path



for f in get_files(directory, 'bag', recursive=True):
  #print("Reading the rosbag file")
  #if not directory.endswith("/"):
  #  directory += "/"
  #extension = ""
  #if not filename.endswith(".bag"):
  #  extension = ".bag"
  #bag = rosbag.Bag(directory + filename + extension)
  bag = rosbag.Bag(f)
  
  # Create directory with name filename (without extension)
  results_dir = directory +'/'+'results_'+ filename[:-4] 
  if not os.path.exists(results_dir):
    os.makedirs(results_dir)
    
  print("Writing robot joint state data to CSV")
  force = [0,0,0,0,0,0]
  lastsecond = None
  print("subdir", subdir)
  with open(results_dir +"/"+filename+subdir+'_forcetorque-calibrated.csv', mode='w') as data_file:
    data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    data_writer.writerow(['time', 'forcex', 'forcey','forcez','torquex','torquey','torquez'])
    # Get all message on the /ft_sensor/ft_compensated topic
    count = 1
    for topic, msg, t in bag.read_messages(topics=['/ft_sensor/ft_compensated']):
        t_epoch = msg.header.stamp.secs
        
        if time.localtime(t_epoch)[5] != lastsecond:
          count = 1
          lastsecond = time.localtime(t_epoch)[5]
        else:
          count = count+1
          t = time.strftime("%a, %d %b %Y %H:%M:%S",
                            time.localtime(t_epoch))
          t +='.'+str(count)
          force[0]=msg.wrench.force.x
          force[1]=msg.wrench.force.y
          force[2]=msg.wrench.force.z
          force[3]=msg.wrench.torque.x
          force[4]=msg.wrench.torque.y
          force[5]=msg.wrench.torque.z
          data_writer.writerow([t, force[0], force[1], force[2], force[3], force[4], force[5]])
          
  print("Finished creating csv file!")
  bag.close()
