
import time
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import gc
import board
import storage
#from machine import I2C, Pin, Timer


def IMUrecord(IMUSTOP=False, filename='IMU_readings.csv'): 
    # Define the IMU 
    #lsm = LSM6DSOX(I2C(0, scl=Pin(13), sda=Pin(12)))
    lsm = board.I2C()  # uses board.SCL and board.SDA
    sensor = LSM6DSOX(lsm)
    
    # open the file in "append" mode so we don't overwrite prior runs
    filewrite = open(filename, "a")

    # set a header to connect data to experiment realtime
    today = time.localtime()
    print(today) # printing to the REPL/terminal
    print("Current Time/Date", today, file=filewrite) # print the header into the file

    time.sleep_ms(500) # IMU startup time

#    IMUdatacollect = []
    timecount = 0
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
        timecount = timecount+1



    filewrite.flush()

if __name__ == "__main__":
    print("IMU Recording")
    IMUrecord()
    
