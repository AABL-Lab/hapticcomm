import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_ntp import NTP


def set_ntp_time(esp):
# Initialize the NTP o  bject
    ntp = NTP(esp)

    # Fetch and set the microcontroller's current UTC time
# keep retrying until a valid time is returned
    while not ntp.valid_time:
        ntp.set_time()
        print("Failed to obtain time, retrying in 5 seconds...")
        time.sleep(5)# Write your code here :-)
