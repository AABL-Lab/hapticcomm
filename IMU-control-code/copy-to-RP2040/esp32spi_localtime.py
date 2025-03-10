# SPDX-FileCopyrightText: 2019 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import busio
from digitalio import DigitalInOut
import rtc
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import adafruit_esp32spi_wifimanager


TIME_API = "http://worldtimeapi.org/api/ip"

def jsontime(wifi):

    the_rtc = rtc.RTC()

    response = None
    while True:
        try:
            print("Fetching json from", TIME_API)
            response = wifi.get(TIME_API)
            break
        except OSError as e:
            print("Failed to get data, retrying\n", e)
            continue

    json = response.json()
    current_time = json["datetime"]
    the_date, the_time = current_time.split("T")
    year, month, mday = [int(x) for x in the_date.split("-")]
    the_time = the_time.split(".")[0]
    hours, minutes, seconds = [int(x) for x in the_time.split(":")]

    # We can also fill in these extra nice things
    year_day = json["day_of_year"]
    week_day = json["day_of_week"]
    is_dst = json["dst"]

    now = time.struct_time(
        (year, month, mday, hours, minutes, seconds, week_day, year_day, is_dst)
    )
    print(now)
    the_rtc.datetime = now


    return now
