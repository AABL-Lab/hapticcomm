#!/usr/bin/env python3
import subprocess
import re
import math
import csv
from optparse import OptionParser
length_regexp = 'Duration: (\d{2}):(\d{2}):(\d{2})\.\d+,'
re_length = re.compile(length_regexp)
import datetime
import sys
def main(filename=None, CSVfilename=None):
    if filename==None:
        print("Filename to split?")
        filename = str(input())
        print("CSV with split times")
        CSVfilename = str(input())

    print("Loading split times from", CSVfilename)
    cliptimes = []
    try:
        with open(CSVfilename, 'r') as f:
            for line in f:
                print(line)
                if line.startswith("UTCtime"):
                    # pull out just the time
                    linelabel,linetime,linename = line.split('**')
                    linename = linename.replace(",","")
                    print("start at", linetime, "name", linename)
                    # convert from string format to datetime format
                    format = '%Y,%m,%d,%H:%M:%S'
                    thislinetime = datetime.datetime.strptime(linetime, format)
                    cliptimes.append([thislinetime,linename])

    except Exception as e:
        print(e)
    
    if split_length <= 0:
        print("Split length can't be 0")
        raise SystemExit
    output = subprocess.Popen("ffmpeg -i '"+filename+"' 2>&1 | grep 'Duration'",
                              shell = True,
                              stdout = subprocess.PIPE
    ).stdout.read()
    print(output)
    matches = re_length.search(output)
    if matches:
        video_length = int(matches.group(1)) * 3600 + \
                       int(matches.group(2)) * 60 + \
                       int(matches.group(3))
        print("Video length in seconds: "+str(video_length))
    else:
        print("Can't determine video length.")
        raise SystemExit
    split_count = int(math.ceil(video_length/float(split_length)))
    if(split_count == 1):
        print("Video length is less then the target split length.")
        raise SystemExit
    split_cmd = "ffmpeg -i '"+filename+"' -vcodec copy "
    for n in range(0, split_count):
        split_str = ""
        if n == 0:
            split_start = 0
        else:
            split_start = split_length * n
            split_str += " -ss "+str(split_start)+" -t "+str(split_length) + \
                         " '"+filename[:-4] + "-" + str(n) + "." + filename[-3:] + \
                         "'"
    print("About to run: "+split_cmd+split_str)
    output = subprocess.Popen(split_cmd+split_str, shell = True, stdout =
                              subprocess.PIPE).stdout.read()

if __name__ == '__main__':
    try:
        filename = sys.argv[1]
        CSVfilename = sys.argv[2]
    except Exception as e:
        print(e)
        filename=None
        CSVfilename=None
    try:
        main(filename,CSVfilename)
    except Exception as e:
        print("Exception occured running main():")
        print(str(e))
