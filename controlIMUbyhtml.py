#wifitest.py
 # SPDX-FileCopyrightText: 2021 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

'''Adapted from the Adafruit_CircuitPython_ESP32SPI
library example esp32spi_simpletest.py:
https://github.com/adafruit/Adafruit_CircuitPython_ESP32SPI/
blob/master/examples/esp32spi_simpletest.py '''

import board
import busio
from digitalio import DigitalInOut
import adafruit_requests as requests
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from adafruit_esp32spi import adafruit_esp32spi


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

requests.set_socket(socket, esp)

if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
    print("ESP32 found and in idle mode")
print("Firmware vers.", esp.firmware_version)
print("MAC addr:", [hex(i) for i in esp.MAC_address])

for ap in esp.scan_networks():
    print("\t%s\t\tRSSI: %d" % (str(ap['ssid'], 'utf-8'), ap['rssi']))

print("Connecting to AP...")
while not esp.is_connected:
    try:
        esp.connect_AP(secrets["ssid"], secrets["password"])
    except RuntimeError as e:
        print("could not connect to AP, retrying: ", e)
        continue
print("Connected to", str(esp.ssid, "utf-8"), "\tRSSI:", esp.rssi)
print("My IP address is", esp.pretty_ip(esp.ip_address))


def web_page():
  if led.value() == 1:
    gpio_state="ON"
  else:
    gpio_state="OFF"
  
  html = open(mainpage.html)
  return html
  
try:
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  #s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  s.connect(('', 80))
  s.recv(5)
except OSError as e:
  machine.reset()
timecount = 0
while True:
  try:
    if gc.mem_free() < 102000:
      gc.collect()
    conn, addr = s.accept()
    conn.settimeout(3.0)
    print('Got a connection from %s' % str(addr))
    request = conn.recv(1024)
    conn.settimeout(None)
    request = str(request)
    print('Content = %s' % request)
    IMU_start = request.find('/?IMU=recording') 
    IMU_stop = request.find('/?IMU=stopped')
    if IMU_start == 6:
      print('STARTING IMU RECORDING')
      led.value(1)
      IMUSTOP = False
      print(IMUSTOP)
      IMUrecord(timecount, IMUSTOP)

    if IMU_stop == 6:
      print('STOPPING IMU RECORDING')
      led.value(0)
      IMUSTOP = True
      print(IMUSTOP)
      IMUrecord(timecount, IMUSTOP)
      timecount = 0 #reset the timer for next recording


    response = web_page()
    conn.send('HTTP/1.1 200 OK\n')
    conn.send('Content-Type: text/html\n')
    conn.send('Connection: close\n\n')
    conn.sendall(response)
    conn.close()
  except OSError as e:
    conn.close()
    print('Connection closed')
    

print("Done!")