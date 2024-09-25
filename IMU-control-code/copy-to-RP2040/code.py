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
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
import adafruit_esp32spi.adafruit_esp32spi_wsgiserver as wsgi
from adafruit_esp32spi import adafruit_esp32spi
import adafruit_esp32spi.adafruit_esp32spi_wsgiserver as server
from adafruit_wsgi.wsgi_app import WSGIApp
import adafruit_connection_manager
import IMU
import os
import rtc
import time
import struct
import esp32spi_localtime
from adafruit_esp32spi import adafruit_esp32spi_wifimanager

magenta= (255,0,0)
pink = (255,128,0)
red = (255,255,51)
green = (0,255,255)
blue = (0,128,255)
light_blue = (102,0,204)
yellow = (0,255,0)
purple = (255,0,100)
orange = (100,100,51)

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

adafruit_requests.set_socket(socket, esp)

if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
    print("ESP32 found and in idle mode")
print("Firmware vers.", esp.firmware_version)
print("MAC addr:", [hex(i) for i in esp.MAC_address])

import adafruit_rgbled
from adafruit_esp32spi import PWMOut
RED_LED = PWMOut.PWMOut(esp, 25)
GREEN_LED = PWMOut.PWMOut(esp, 27)
BLUE_LED = PWMOut.PWMOut(esp, 26)
status_light = adafruit_rgbled.RGBLED(RED_LED, BLUE_LED, GREEN_LED)
wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets, status_light)
wifi.connect()

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
        now = esp32spi_localtime.jsontime(wifi)
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



class SimpleWSGIApplication:
    """
    An example of a simple WSGI Application that supports
    basic route handling and static asset file serving for common file types
    """

    INDEX = "/index.html"
    CHUNK_SIZE = 8912  # max number of bytes to read at once when reading files

    def __init__(self, static_dir=None, debug=False):
        self._debug = debug
        self._listeners = {}
        self._start_response = None
        self._static = static_dir
        if self._static:
            self._static_files = ["/" + file for file in os.listdir(self._static)]

    def __call__(self, environ, start_response):
        """
        Called whenever the server gets a request.
        The environ dict has details about the request per wsgi specification.
        Call start_response with the response status string and headers as a list of tuples.
        Return a single item list with the item being your response data string.
        """
        if self._debug:
            self._log_environ(environ)

        self._start_response = start_response
        status = ""
        headers = []
        resp_data = []

        key = self._get_listener_key(
            environ["REQUEST_METHOD"].lower(), environ["PATH_INFO"]
        )
        if key in self._listeners:
            status, headers, resp_data = self._listeners[key](environ)
        if environ["REQUEST_METHOD"].lower() == "get" and self._static:
            path = environ["PATH_INFO"]
            if path in self._static_files:
                status, headers, resp_data = self.serve_file(
                    path, directory=self._static
                )
            elif path == "/" and self.INDEX in self._static_files:
                status, headers, resp_data = self.serve_file(
                    self.INDEX, directory=self._static
                )

        self._start_response(status, headers)
        return resp_data

    def on(self, method, path, request_handler):
        """
        Register a Request Handler for a particular HTTP method and path.
        request_handler will be called whenever a matching HTTP request is received.
        request_handler should accept the following args:
            (Dict environ)
        request_handler should return a tuple in the shape of:
            (status, header_list, data_iterable)
        :param str method: the method of the HTTP request
        :param str path: the path of the HTTP request
        :param func request_handler: the function to call
        """
        self._listeners[self._get_listener_key(method, path)] = request_handler

    def serve_file(self, file_path, directory=None):
        status = "200 OK"
        print("served file",file_path)
        headers = [("Content-Type", self._get_content_type(file_path))]

        full_path = file_path if not directory else directory + file_path

        def resp_iter():
            with open(full_path, "rb") as file:
                while True:
                    chunk = file.read(self.CHUNK_SIZE)
                    if chunk:
                        yield chunk
                    else:
                        break

        return (status, headers, resp_iter())

    def _log_environ(self, environ):  # pylint: disable=no-self-use
        print("environ map:")
        for name, value in environ.items():
            print(name, value)

    def _get_listener_key(self, method, path):  # pylint: disable=no-self-use
        return "{0}|{1}".format(method.lower(), path)

    def _get_content_type(self, file):  # pylint: disable=no-self-use
        ext = file.split(".")[-1]
        if ext in ("html", "htm"):
            return "text/html"
        if ext == "js":
            return "application/javascript"
        if ext == "css":
            return "text/css"
        if ext in ("jpg", "jpeg"):
            return "image/jpeg"
        if ext == "png":
            return "image/png"
        return "text/plain"


# Our HTTP Request handlers
def led_on(environ):  # pylint: disable=unused-argument
    print("LED on!")
    status_light.color = green
    return web_app.serve_file("static/index.html")

def led_off(environ):  # pylint: disable=unused-argument
    print("LED off!")
    status_light.color = (255,255,255)
    return web_app.serve_file("static/index.html")

def IMU_on(environ): # starts the IMU recording
    global IMU_recording
    IMU_recording = True
    status_light.color = light_blue
    print("IMU recording")
    datenow = rtc.RTC().datetime
    stringdate = "UTCtime:**"+str(datenow[0])+","+str(datenow[1])+","+str(datenow[2])+","+str(datenow[3])+":"+str(datenow[4])+":"+str(datenow[5])
    print("current date/time is ", stringdate)
    with open(filename, "a") as fp:
        # print the header into the file
        fp.write(stringdate+"\n")
        fp.flush()
    return web_app.serve_file("static/IMUon.html")

def IMU_off(environ):
    global IMU_recording
    IMU_recording = False
    status_light.color = orange
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
    return web_app.serve_file("static/IMUoff.html")

def preview_last_data(environ):  # pylint: disable=unused-argument
    print("previewing")
    status_light.color = purple
    return web_app.serve_file("IMU_readings.csv")


# Here we create our application, setting the static directory location
# and registering the above request_handlers for specific HTTP requests
# we want to listen and respond to.
static = "/static"
try:
    static_files = os.listdir(static)
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

web_app = SimpleWSGIApplication(static_dir=static)
web_app.on("GET", "/", led_on)
web_app.on("GET", "/IMU_on", IMU_on)
web_app.on("GET", "/IMU_off", IMU_off)
web_app.on("GET", "/preview_last_data", preview_last_data)


# Here we setup our server, passing in our web_app as the application
server.set_interface(esp)
wsgiServer = server.WSGIServer(80, application=web_app)
status_light.color= green
print("open this IP in your browser: ", esp.pretty_ip(esp.ip_address))

# initialize the IMU recording and IMU timecount global,
#which will be updated inside the IMU control functions
# but can't be passed back out by the webserver
IMU_recording = False
timecount = 0
IMU_data = []
# open the file in "append" mode so we don't overwrite prior runs
filename='IMU_readings.csv'
try:
    with open(filename, "a") as fp:
        # print the header into the file
        fp.write("IMU restart"+"\n")
        fp.flush()
except Exception as e:
    print("Not able to write to ", filename, "error", e)
    status_light.color=red
timenow = rtc.RTC().datetime
print("server should be ready") # printing to the REPL/terminal

# Start the webserver
wsgiServer.start()
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
                print("there is a problem")
                IMU_off(argument) # what is the argument? This currently crashes

    except OSError as e:
        print("Failed to update server, restarting ESP32\n", e)
        # put something here to actually reset the ESP32
        supervisor.reload()
