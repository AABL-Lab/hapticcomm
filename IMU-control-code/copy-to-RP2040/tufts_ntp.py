# SPDX-FileCopyrightText: 2021 Adafruit Industries
# SPDX-License-Identifier: MIT

import struct
import time
import board
import busio
import rtc
import time
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from adafruit_ntp import NTP

TIMEOUT = 5
# edit host and port to match server
HOST = "time.eecs.tufts.edu"
PORT = 123
# 1970-01-01 00:00:00
NTP_TO_UNIX_EPOCH = 2208988800

def set_ntp_time(esp):
    # create the socket
    socket.set_interface(esp)
    s = socket.socket()

    s.settimeout(TIMEOUT)
    # test for connectivity to server
    print("Server ping:", esp.ping(HOST), "ms to host ", HOST)
    print("Sending")

    data = b'\x1b' + 47 * b'\0'
    s.connect((HOST, 123))
    s.send(data) #, (HOST, PORT))
    data, address = esp.recv(1024)
    if data:
        t = struct.unpack('!12I', data)[10]
        #t -= NTP_TO_UNIX_EPOCH

    print("Time:", time.localtime(t - NTP_TO_UNIX_EPOCH))

    # convert seconds to the struct that rtc.datetime needs
    currenttime = time.localtime(t - NTP_TO_UNIX_EPOCH)

    # now update the time on the board


    r = rtc.RTC()
    r.datetime = time.struct_time(currenttime)
    print("\n\n system time set, RTC.datetime =", r.datetime)
    s.close()
