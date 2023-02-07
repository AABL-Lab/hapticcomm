
import time
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import gc
import board
import storage
print("The IMU module requires that pin D2 is shorted to ground to make the R2040 writeable")

def IMUrecord(timecount): 
    # Define the IMU 
    #sensor = sensor6DSOX(I2C(0, scl=Pin(13), sda=Pin(12)))
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = LSM6DSOX(i2c)


#    IMUdatacollect = [] 
        # Collecting each of the values separately to make them easier to write/read as CSV
    IMUDataX = sensor.acceleration[0] # X acceleration data in m/s^2
    IMUDataY = sensor.acceleration[1] # Y acceleration data in m/s^2
    IMUDataZ = sensor.acceleration[2] # Z acceleration data in m/s^2
    GyroDataX = sensor.gyro[0] # gyro x in radians/second
    GyroDataY = sensor.gyro[1] # gyro Y in radians/second
    GyroDataZ = sensor.gyro[2] # gyro Z in radians/second
    #    print("Gyro data", GyroData)
    #    print("Acceleration data from timestep", timecount, "is", IMUdata)
        
    # printing to file in CSV format, which we have to do by hand because micropython/circuitpython
    row = (timecount,IMUDataX, IMUDataY,IMUDataZ,GyroDataX,GyroDataY,GyroDataZ)
    #print("printed to CSV", filename)
    timecount = timecount+1
    return timecount, row



    filewrite.flush()

if __name__ == "__main__":
    print("IMU Recording")
    IMUrun = True
    timecount = 0
    IMU_data = []
    while IMUrun ==True:
        timecount, row = IMUrecord(timecount) # read once from the IMU and write to the file
        #print(timecount)
        #print(row)
        IMU_data.append(row)
        if timecount > 20:
            IMUrun = False
    print(IMU_data)
        
