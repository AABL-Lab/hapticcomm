
import time
from lsm6dsox import LSM6DSOX
import gc
from machine import I2C, Pin, Timer


class DataCollection: 
    # Define the IMU 
    lsm = LSM6DSOX(I2C(0, scl=Pin(13), sda=Pin(12)))
    def __init__(self, duration=1000, filename="IMU_readings.csv"):
        self.duration = duration  # set how long the IMU will record into the file, default 10 seconds
        self.filename = filename # set where the IMU will record, default is IMU_readings.csv 

    # open the file in "append" mode so we don't overwrite prior runs
    filewrite = open(self.filename, "a")

    # set a header to connect data to experiment realtime
    today = time.localtime()
    print(today) # printing to the REPL/terminal
    print("Current Time/Date", today, file=filewrite) # print the header into the file

    time.sleep_ms(500) # IMU startup time


    def timeoutcallback(t): # when the timer hits the time set, stop the IMU
        IMUSTOP==True

    # one shot timer, firing after self.duration ms, using machine.Timer
    IMUtimer.init(mode=Timer.ONE_SHOT, period=self.duration, callback=timeoutcallback)

#    IMUdatacollect = []
    while not IMUSTOP: 
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



    filewrite.flush()


