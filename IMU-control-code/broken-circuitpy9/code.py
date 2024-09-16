# Kat Allen kat.allen@tufts.edu
# Circuitpython for controlling an RP2040 Nano Connect IMU via web browser/wifi

'''Thank you to
https://learn.adafruit.com/circuitpython-on-the-arduino-nano-rp2040-connect/wifi
and the Adafruit Discord community!
'''

import board
import supervisor
import busio
from digitalio import DigitalInOut, Direction
import adafruit_requests
from adafruit_esp32spi import adafruit_esp32spi, adafruit_esp32spi_wifimanager
import adafruit_connection_manager
import IMU
import os
import rtc
import time
import struct
import getJSONtime
import adafruit_esp32spi.adafruit_esp32spi_socketpool as socketpool
import esp32spi_wsgiserver as server
import wsgiserver


# Get wifi details and more from a secrets.py file
try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

print("Arduino Nano RP2040 Connect webclient test")

TEXT_URL = "http://wifitest.adafruit.com/testwifi/index.html"

#  ESP32 pins
esp32_cs = DigitalInOut(board.CS1)
esp32_ready = DigitalInOut(board.ESP_BUSY)
esp32_reset = DigitalInOut(board.ESP_RESET)

#  uses the secondary SPI connected through the ESP32
spi = busio.SPI(board.SCK1, board.MOSI1, board.MISO1)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

# LED setup for onboard LED
import adafruit_rgbled
from adafruit_esp32spi import PWMOut
RED_LED = PWMOut.PWMOut(esp, 26)
GREEN_LED = PWMOut.PWMOut(esp, 27)
BLUE_LED = PWMOut.PWMOut(esp, 25)
status_light = adafruit_rgbled.RGBLED(RED_LED, BLUE_LED, GREEN_LED)

wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets, status_light)



if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
    print("ESP32 found and in idle mode")
print("Firmware vers.", esp.firmware_version)
print("MAC addr:", [hex(i) for i in esp.MAC_address])

import adafruit_rgbled
from adafruit_esp32spi import PWMOut
RED_LED = PWMOut.PWMOut(esp, 26)
GREEN_LED = PWMOut.PWMOut(esp, 27)
BLUE_LED = PWMOut.PWMOut(esp, 25)
status_light = adafruit_rgbled.RGBLED(RED_LED, BLUE_LED, GREEN_LED)
wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets, status_light)
wifi.connect()
# print("Connecting to AP...")
# while not esp.is_connected:
#     try:
#         esp.connect_AP(secrets["ssid"], secrets["password"])
#     except RuntimeError as e:
#         print("could not connect to AP, retrying: ", e)
#         continue
print("My IP address is", esp.pretty_ip(esp.ip_address))

if secrets["ssid"]!="tufts_eecs":
    # now sync with NTP time
    TZ_OFFSET = 3600 * 2
    def try_set_time(esp, tz_offset=0):
        # get_time will raise ValueError if the time isn't available yet.
        try:
            now = time.localtime(esp.get_time()[0] + tz_offset)
        except OSError:
            return False
        rtc.RTC().datetime = now
        print("got time")
        return True

    while not try_set_time(esp, TZ_OFFSET):
        time.sleep(0.01)
else:
    try:
        #ntp = tufts_ntp.set_ntp_time(esp)
        #ntp = adafruit_ntp.NTP(pool, tz_offset=0)
        now = getJSONtime.getJSONtime(wifi)
        print("Got time from JSON")
    except Exception as e:
 #      manually set the time
        print("error:", e)
        print("Manually setting time")
        now =  time.struct_time((2023, 2, 7, 13, 43, 15, 0, -1, -1))
        rtc.RTC().datetime =  now #time.struct_time((2019, 5, 29, 15, 14, 15, 0, -1, -1))



# Pretty-parse the struct_time
print("It is currently {}/{}/{} at {}:{}:{} UTC".format(
    now.tm_mon, now.tm_mday, now.tm_year,
    now.tm_hour,now.tm_min, now.tm_sec))

static = "/static"
pool = adafruit_connection_manager.get_radio_socketpool(esp)
print("open this IP in your browser: ", esp.pretty_ip(esp.ip_address))
web_app = wsgiserver.SimpleWSGIApplication(static_dir=static)


# Here we setup our server, passing in our web_app as the application
server.set_interface(esp)
wsgiServer = server.WSGIServer(80, application=web_app)

# Our HTTP Request handlers

@web_app.route("/")
def base(request: Request):
    """
    Serve the default index.html file.
    """
    return FileResponse(request, "static/index.html")

@web_app.route("/led_on/<r>/<g>/<b>")
def led_on(request, r, g, b):  # pylint: disable=unused-argument
    print("LED on!")
    status_light((int(r), int(g), int(b)))
    return FileResponse(request, "static/index.html")

@web_app.route("/led_off")
def led_off(request):  # pylint: disable=unused-argument
    print("LED off!")
    status_light(0,0,0)
    return FileResponse(request, "static/index.html")

@web_app.route("/imu_on")
def IMU_on(request): # starts the IMU recording
    global IMU_recording
    IMU_recording = True
    status_light.value = True
    print("IMU recording")
    datenow = rtc.RTC().datetime
    stringdate = "UTCtime:"+str(datenow[0])+","+str(datenow[1])+","+str(datenow[2])+","+str(datenow[3])+":"+str(datenow[4])+":"+str(datenow[5])
    print("current date/time is ", stringdate)
    with open(filename, "a") as fp:
        # print the header into the file
        fp.write(stringdate+"\n")
        fp.flush()
    return FileResponse(request, "static/IMUon.html")


@web_app.route("/imu_off")
def IMU_off(request):
    global IMU_recording
    IMU_recording = False
    status_light.value = False
    print("IMU stopped")
    with open(filename, "a") as fp:
        for row in IMU_data: # send each row in the IMU data to the file
            #print(row, "writing to file", filename)
            fp.write(str(row)+"\n")
            fp.flush()
        fp.write("end of logging\n")
        fp.flush()
    print("Wrote to file")
    global timecount
    timecount = 0 # reset the global counter
    return FileResponse(request, "static/IMUoff.html")

# Here we create our application, setting the static directory location
# and registering the above request_handlers for specific HTTP requests
# we want to listen and respond to.
web_app.on("GET", "/IMU_on", IMU_on)
web_app.on("GET", "/IMU_off", IMU_off)

try:
    static_files = os.listdir(static)
    print(static_files)
    if "index.html" not in static_files:
        raise RuntimeError(
            """
            This webserver depends on an index.html, but it isn't present.
            Please add it to the {0} directory""".format(
                static
            )
        )
except OSError as e:
    raise RuntimeError(
        """
        This webserver depends on a static asset directory.
        Please create one named {0} in the root of the device filesystem.""".format(
            static
        )
    ) from e







# initialize the IMU recording and IMU timecount global,
#which will be updated inside the IMU control functions
# but can't be passed back out by the webserver
IMU_recording = False
timecount = 0
IMU_data = []
# open the file in "append" mode so we don't overwrite prior runs
filename='IMU_readings.csv'
today = rtc.RTC().datetime

while True:
    # Our main loop where we have the server poll for incoming requests
    #print("*")
    try:
        wsgiServer.update_poll()
        # background tasks (like reading the IMU)
        if IMU_recording == True:
            if timecount < 200:
                try:
                    timecount, row = IMU.IMUrecord(timecount) # read once from the IMU and write to the file
                    print(timecount)
                #print(row)
                except:
                    row = "error"
                IMU_data.append(row)
            else: # running out of memory
                IMU_off(argument) # what is the argument? This currently crashes
        # Process any waiting requests

        if pool_result == REQUEST_HANDLED_RESPONSE_SENT:
            # Do something only after handling a request
            pass
    except (ValueError, RuntimeError, OSError) as e:
        print("Failed to update server, restarting ESP32\n", e)
        # put something here to actually reset the ESP32
        wifi.reset()
        continue
