import matplotlib.pyplot as plt
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
import numpy as np
import glob
import os
import sys
from datetime import datetime, timedelta, date
import itertools

import re

filename = "group0-test-test.csv"


# find the beginning of the actual IMU data and strip off the start time, removing any prior lines
with open(filename, 'r') as f:
    first_line = f.readline().strip()
    while first_line.split(',')[0][:3]!="UTC":
        first_line = f.readline().strip() # strip off the next line

# Extract clocktimeIMUstart as a datetime object from that first line
print(first_line, "is the first line of the IMU csv")
clocktimeIMUstart_str = first_line.split(',')[3]
match = re.search(r'(\d{1,2}):(\d{1,2}):(\d{1,2})', clocktimeIMUstart_str)
if match:
    clocktimeIMUstart = datetime.strptime(match.group(), "%H:%M:%S").time()

    df = pd.read_csv(filename, skiprows=1, names=["timestamp", "Xaccel", "Yaccel", "Zaccel", "Xrotation", "Yrotation", "Zrotation"], engine='python')
    df = df.applymap(lambda x: str(x).replace('(', '').replace(')', ''))

    end_row = df[df.apply(lambda row: row.astype(str).str.contains('end').any(), axis=1)]

    # Check if 'end' row is found
if not end_row.empty:
    # Combine columns that hold the timestamp (for example, columns 0, 1, 2 might hold timestamp info)
    # Adjust column indices based on where the timestamp is split in your case
    timestamp_parts = end_row.iloc[0, :5].astype(str).apply(lambda x: x.replace('(', '').replace(')', ''))
    
    # Combine parts of the timestamp into one string
    timestamp_str = ' '.join(timestamp_parts)
    print(timestamp_str)

    # Now convert the string to datetime (adjust format if needed)
    match = re.search(r'(\d{1,2}):(\d{1,2}):(\d{1,2})',timestamp_str)
    if match:
        end_timestamp = datetime.strptime(match.group(), "%H:%M:%S").time()

    print(f"Extracted timestamp: {end_timestamp}")
else:
    print("No 'end' row found.")
    
fileduration =  datetime.combine(date.min,end_timestamp)-  datetime.combine(date.min, clocktimeIMUstart)
print(fileduration , " is the duration of the IMU log")


# Identify rows that have errors in conversion (but do not drop them yet)
error_rows = df.apply(lambda row: any("error" in str(x).lower() for x in row), axis=1)
# Convert to numeric and coerce errors (errors will be NaN)
df = df.apply(pd.to_numeric, errors='coerce')
# Drop rows that contain NaN values (this includes the original error rows)
df = df.dropna()

df.reset_index(drop=True, inplace=True)

# Renumber the IMU datapoints so that they are continuous
df['timestamp'] = range(1, len(df) + 1)


# pull out labels for this data from the IMU filename before we add it to our list of dataframes to plot
metadata = filename.split('/')[-1].split('-')
group, subgroup, card = metadata[0].replace('group', ''), metadata[1], metadata[2].split('.')[0]
print(f"Group: {group}, Subgroup: {subgroup}, Card: {card}")

# calculate timestamps per second
tps = fileduration.seconds/len(df)
print("There are ", tps ," IMU recordings per second")
