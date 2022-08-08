# Rename to main.py to run automatically on boot

import time
from lsm6dsox import LSM6DSOX
import gc
from machine import I2C, Pin


# Define the IMU 
lsm = LSM6DSOX(I2C(0, scl=Pin(13), sda=Pin(12)))
zsample = [0]*300
ysample = [0]*300
xsample = [0]*300

# open the file in "append" mode so we don't overwrite prior runs
filewrite = open("IMU_readings.csv", "a")
# set a header to connect data to experiment realtime
today = time.localtime()
print(today)
print("Current Time/Date", today, file=filewrite)

time.sleep_ms(500) # IMU startup time
timecount = 0 # approximately ms
IMUdatacollect = []
while (timecount < 7000): #this does not have real units but about 2*ms runtime
    gc.collect() # don't remember why we need to garbage collect every time
    
    # Collecting each of the values separately to make them easier to write/read as CSV
    IMUDataX = lsm.read_accel()[0] # X acceleration data in m/s^2
    IMUDataY = lsm.read_accel()[1] # Y acceleration data in m/s^2
    IMUDataZ = lsm.read_accel()[2] # Z acceleration data in m/s^2
    GyroDataX = lsm.read_gyro()[0] # gyro x in radians/second
    GyroDataY = lsm.read_gyro()[0] # gyro Y in radians/second
    GyroDataZ = lsm.read_gyro()[0] # gyro Z in radians/second
#    print("Gyro data", GyroData)
#    print("Acceleration data from timestep", timecount, "is", IMUdata)

# printing to file in CSV format, which we have to do by hand because micropython
    print(timecount,",",IMUDataX,",", IMUDataY,",", IMUDataZ,",",  GyroDataX,",",  GyroDataY,",",  GyroDataZ, file=filewrite)
    time.sleep_ms(10)
    timecount = timecount+10



filewrite.flush()


