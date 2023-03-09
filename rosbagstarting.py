import rosbag_recorder # fix to give full path

# filename param  doesn't exist yet, Reuben is updating
# will throw an error if topics do not exist
recorder = RosbagRecorder(logging_directory, ['/joint_states', '/tf'], filename)
                                              # look up all the topics I want with rostopic list

recorder.start()

# needs to be manually stopped
# consider putting the whole trial in a try/except and then
finally:
    recorder.stop()

