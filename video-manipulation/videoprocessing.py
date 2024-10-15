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


def main(filename, CSVfilename, experimentnumber):
    print("Loading split times from", CSVfilename)
    cliptimes = []
    try:
        with open(CSVfilename, 'r') as f:
            for line in f:
                print(line)
                if line.startswith("UTCtime"):
                    # pull out just the time
                    linelabel,linetime,linename = line.split('**')
                    # strip extra commas from the name
                    linename = linename.replace(",","")
                    linename = linename.replace("\n","-")
                    # strip spaced from the time
                    linetime = linetime.replace(" ","")
                    #print("start at", linetime, "name", linename)
                    # convert from string format to datetime format
                    print("Line time is: ", linetime)
                    format = '%Y,%m,%d,%H:%M:%S'
                    thislinetime = datetime.datetime.strptime(linetime, format)
                    cliptimes.append([thislinetime,linename])

    except Exception as e:
        print("an exception occurred", e)

    print("all clip times", cliptimes)
    print("start time for this video is: ", cliptimes[0])
    videostarttime = cliptimes[0][0]
    lastcliptime = videostarttime
    for cliptime in cliptimes:
        print("splitting off clip", cliptime[1], "utctime: ", cliptime[0])
        timedelta = cliptime[0]-videostarttime
        print(timedelta, "is the time from video start for clip", cliptime[1])
        
        # building up this shape of command:        
        # ffmpeg -i topcamera.mkv -ss 00:00:30 -t 90 -c:v copy -c:a copy hleadertopcamera.mkv
        outputfilename = cliptime[1]+filename
        clipduration = (cliptime[0]-lastcliptime).total_seconds()
        print("clip duration is ", clipduration)
        if clipduration !=0: 
            split_cmd = "ffmpeg -i "+ filename+ " -ss "+ str(timedelta)+" -t "+ str(clipduration) + " -c:v copy -c:a copy " +outputfilename
            print("About to run: ", split_cmd)
            input()
            output = subprocess.Popen(split_cmd, shell = True, stdout =
                                          subprocess.PIPE).stdout.read()
        lastcliptime = cliptime[0] 

        
def get_cardorder(experimentnumber=""):
    if experimentnumber == "":
        got_experimentnumber=False
        while not got_experimentnumber:
            print("Experiment Number:")
            try:
                experimentnumber = int(input())
                got_experimentnumber = True
            except:
                print("Experiment Number (integer)")
                experimentnumber = int(input())
    try:
        filename = "/home/katallen/workspace/src/hapticcomm/experiment_card_full_order.txt"
        cardset = experimentnumber % 6
        print("cardset is ", cardset)
        print("Using Card Set", cardset)
        with open(filename, 'r') as f:
            filelist = csv.DictReader(f)    
            cardsorts = [row for row in filelist] #all the card orders
            # just this experiment's cards
            card_dictionary = cardsorts[cardset-1]
            cards = list(card_dictionary.values()) 
            print("\n\n\nThis experiment's cards are, ", cards)
            return cards
    except Exception as e: 
        print("cards not set, ", e)
        exit


def add_cardnames_to_csv(CSVfilename, cards):
    count = 0
    newCSVfilename = "marked"+CSVfilename
    with open(CSVfilename, 'r') as f:
        for line in f:
            if line.startswith("UTCtime"):
                print(line)
                print(cards)
                line = line+cards[count]
                count = (count+1) % 6
            with open(newCSVfilename, 'w') as g:
                writer_object = csv.writer(g, delimiter=',')
                print("The line about to be written is", line)
                writer_object.writerow(line)



    
if __name__ == '__main__':
    try:
        filename = sys.argv[1]
        CSVfilename = sys.argv[2]
        experimentnumber = sys.argv[3]
    except Exception as e:
        print(e)
        print("Filename to split?")
        filename = str(input())
        print("CSV with split times")
        CSVfilename = str(input())
        print("Experiment Number?")
        experimentnumber = str(input())

    print("add names to CSV(1) or main?(2)")
    choice = int(input())
    if choice == 1:
        cards = get_cardorder()
        add_cardnames_to_csv(CSVfilename, cards)
    try:
        main(filename,CSVfilename, experimentnumber)
    except Exception as e:
        print("Exception occured running main():")
        print(str(e))
