# SPDX-FileCopyrightText: 2021 Adafruit Industries
# SPDX-License-Identifier: MIT

import struct
import time
import board
import busio
import rtc
import time
import adafruit_esp32spi.adafruit_esp32spi_socketpool as socketpool

TIMEOUT = 5
# edit host and port to match server
HOST = "time.eecs.tufts.edu"
PORT = 123
# 1970-01-01 00:00:00
NTP_TO_UNIX_EPOCH = 2208988800

def set_ntp_time(esp):
    # create the socket
    pool = socketpool.SocketPool(esp)

    s.settimeout(TIMEOUT)
    # test for connectivity to server
    print("Server ping:", esp.ping(HOST), "ms")
    print("Sending")
    s.connect(socketaddr, conntype=esp.UDP_MODE)
    packet = bytearray(48)
    packet[0] = 0b00100011   # Not leap second, NTP version 4, Client mode
    s.send(packet)

    print("Receiving")
    packet = s.recv(48)
    seconds = struct.unpack_from("!I", packet, offset=len(packet) - 8)[0]
    print("Time:", time.localtime(seconds - NTP_TO_UNIX_EPOCH))

    # convert seconds to the struct that rtc.datetime needs
    currenttime = time.localtime(seconds - NTP_TO_UNIX_EPOCH)

    # now update the time on the board


    r = rtc.RTC()
    r.datetime = time.struct_time(currenttime)
    print("\n\n system time set, RTC.datetime =", r.datetime)
    s.close()
