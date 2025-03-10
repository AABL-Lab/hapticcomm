import matplotlib.pyplot as plt
import csv
import sys
import os
import glob
import pandas as pd


def get_csv_data(filename, breakpointfilename=None):

     
     timestamp = []
     Xaccel = []
     Yaccel = []
     Zaccel = []
     Xrotation = []
     Yrotation = []
     Zrotation = []
 

# if breakpoint filename is provided, breakpoints and duration will be added to the dictionary, otherwise get them manually
     
     if breakpointfilename==None:
          duration, breakpoints = getmanualsegmentdata()
          breakpointDF=None # so we do not try to overwrite this later
     else:
          breakpoints = []
          breakpointdf = pd.read_csv(breakpointfilename)
          print("df index: ", breakpointdf.info)
          # read in the csv of breakpoint data as a Pandas DataFrame

     with open(filename, newline='') as csvfile:
          IMUreader = csv.reader(csvfile, delimiter=',', quotechar='|')
          started = False
          countershift = 0
          
          res = filename.split('/')
          group =res[-1].split('-')[0].replace('group','')
          subgroup = res[-1].split('-')[1]
          card = res[-1].split('-')[2].split('.')[0]
          print("group is ", group)
          print("subgroup is ", subgroup)
          print("card is ", card)
          
          # This skips the first row of the CSV file.
          next(IMUreader)


          # now we turn the other rows into plottable lists
          for row in IMUreader:
               print(row[0])
               if row[0].startswith('end'):
                    print("Finished the set")
                    break
               elif row[0].startswith('(0'):
                    if started == False:
                         started = True
                    else:
                         countershift = lasttimestamp+1 # fix counter restarts
                         print("counter offset is now ", countershift)
                         #print(row)
               try:
                    row[0] = row[0].replace('(', '')
                    row[6] = row[6].replace(')','')

                    timestamp.append(float(row[0])+countershift)
                    Xaccel.append(float(row[1]))
                    Yaccel.append(float(row[2]))
                    Zaccel.append(float(row[3]))
                    Xrotation.append(float(row[4]))
                    Yrotation.append(float(row[5]))
                    Zrotation.append(float(row[6]))
                    lasttimestamp = float(row[0])+countershift # for fixing counter restarts
               except:
                    print("Row ", row, "not added")


          if breakpointdf is not None:
               # now we look for matching breakpoint data for this row
               print(breakpointdf['Group'])
               df_g = breakpointdf.loc[breakpointdf['Group'] == int(group)]
               df_sg = df_g.loc[df_g['Subgroup']==subgroup]
               df_bp = df_sg.loc[df_sg['CARD']==card]
               breakpoints = df_bp[['Start', 'L1E', "L2E", "L3E", "L4E", "L5E", 'Duration']].values.tolist()
               print("extracted breakpoints", breakpoints)
               # If it is hh data, there will be two lists, we just need it once
               try:
                    breakpoints = [ int(x) for x in breakpoints[0]]
               except Exception as x:
                    print(x)
                    breakpoints = [int(x) for x in breakpoints]
                    
               duration = breakpoints[-1]
                    
               print("extracted breakpoints", breakpoints, "duration", duration)
 
          
                   

          # print("time:", timestamp)
          # print("Xaccel = ", Xaccel)
          # print("Yaccel = ", Yaccel)
          # print("Zaccel = ", Zaccel)
          # print("Xrot = ", Xrotation)
          # print("Yrot = ", Yrotation)
          # print("Zrotation = ", Zrotation)
          


          

          
          filenamedictionary = {'name': subgroup,'time': timestamp, 'Xaccel':Xaccel, 'Yaccel':Yaccel, 'Zaccel':Zaccel, 'Xrotation':Xrotation, 'Yrotation':Yrotation, 'Zrotation':Zrotation, 'breakpoints':breakpoints, 'duration': duration}

          filenamedictionary = timestampcorrection(filenamedictionary, lasttimestamp)
               
          
          
          
          return filenamedictionary


def timestampcorrection(filenamedictionary, lasttimestamp):
     duration = filenamedictionary['duration']
     if float(duration) == 0:
          filenamedictionary['duration']=filenamedictionary['time'][-1]
          print("Duration not recorded, not correcting timestamps to clock time")
          return filenamedictionary
     
     print("Correcting timestamps for provided duration ", duration, " seconds")
     ######Correcting Timestamps with Duration ####
     if duration != None:
          newtimestamp = []

          for time in filenamedictionary['time']:
               timescalefactor = lasttimestamp/duration

               #print("Time scaling by ", timescalefactor , "ticks per second")
               time = time/ timescalefactor
               #print("New time is ", time)
               newtimestamp.append(time)
               
               #print("new times :", newtimestamp)
               filenamedictionary['time'] = newtimestamp
               #print(filenamedictionary)
               
     return filenamedictionary

def plotallthedata(filenamedictionarylist, plottitle):
     figure, axis = plt.subplots(3,1)
     #fig1 = axis[0,0]
     #fig2 = axis[1,0]
     #fig3 = axis[2,0]
     figure.suptitle(plottitle)
#     axis[0].set_title("X Acceleration"+plottitle)

     for set in filenamedictionarylist:
          print("xaxis ", len(set['Xaccel']), "yaxis ", len(set['time']))
          print("timedata", set['time'])
          print("xaxisdata ", set['Xaccel'])
          axis[0].plot(set['time'],set['Xaccel'], label=set['name'])
     # frequency label
     axis[0].set_ylabel('X Acceleration (N)')
     axis[0].set_xlabel('Experiment Time (s)')
     
#     axis[1].set_title("Y Acceleration"+plottitle)
     for set in filenamedictionarylist:
          axis[1].plot(set['time'],set['Yaccel'], label=set['name'])
     # frequency label
     axis[1].set_ylabel('Y Acceleration (N)')
     axis[1].set_xlabel('Experiment Time (s)')
     
#     axis[2].set_title("Z Acceleration"+plottitle)
     for set in filenamedictionarylist:
          axis[2].plot(set['time'], set['Zaccel'], label = set['name'])
     # frequency label
     axis[2].set_ylabel('Z Acceleration (N)')
     axis[2].set_xlabel('Experiment Time (s)')

     for set in filenamedictionarylist:
          if set['breakpoints'] != []:
               for x in set['breakpoints']:
                    axis[0].axvline(x,color = "black")
                    axis[0].axhline(y = 0, color = "black")
                    axis[1].axvline(x,color = "black")
                    axis[1].axhline(y=0,color = "black")
                    axis[2].axvline(x,color = "black")
                    axis[2].axhline(y=9.8,color = "black")
     
     # function to show the plot
     plt.legend(bbox_to_anchor=(1.05, 1))
     plt.tight_layout(pad=1.05)
     plt.show()

def get_files(path, extension, recursive=False):
    """
    A generator of filepaths for each file into path with the target extension.
    If recursive, it will loop over subfolders as well.
    """
    if not recursive:
         print(path)
         for file_path in glob.iglob(path + "/*." + extension):
            yield file_path
    else:
        for root, dirs, files in os.walk(path):
            for file_path in glob.iglob(root + "/*." + extension):
                print(file_path)
                res = file_path.split('/')
                global filename
                global subdir
                filename = res[-1]
                subdir = res[-2]
                print(filename)
                yield file_path

def scalebysegment(filenamedictionary):
     print("###########################Scaling time to match up segments####################")
     print("Segment breakpoints are ", filenamedictionary['breakpoints'], "and trial duration is ", filenamedictionary['duration'])
     print("timestamps in this file are ", filenamedictionary['time'])
     print("################################################################################")
     newtimestamp = []
     timestamps = filenamedictionary["time"]
     lastpoint = 0
     sectionstart = 0
     
     for point in filenamedictionary['breakpoints']:
          print("point is ", point, "and sectionstart is", sectionstart, "and duration is ", filenamedictionary['duration'])
          for time in timestamps:
               #print("time is", time)
               if time < point and time >= lastpoint:
                    newtime = time/point*10 + sectionstart
                    print("In segment ", lastpoint, "=< x < ", point, "the time was ", time, "and the new time is ", newtime)
                    newtimestamp.append(newtime)
               elif point == filenamedictionary['duration'] and time == filenamedictionary['duration']:
                    newtime = time/filenamedictionary['duration']*10 + sectionstart
                    print("Last point/ new end of trial is ", newtime)
                    newtimestamp.append(newtime)

          lastpoint = point # advance to the next point
          if point !=0:
               sectionstart = sectionstart+10
                    
     lastpoint = 0 #reset last point for new timestamp
     sectionstart = 0 # reset section start for new timestamp
                    
     filenamedictionary['time'] = newtimestamp
     #print(newtimestamp, 'are the new times')
     print("filenamedictionary", filenamedictionary['name'], "has ", len(filenamedictionary['time']), "times and ", len(filenamedictionary['Xaccel']), "values")
     #print(filenamedictionary['time'], "are the times")
     return filenamedictionary



def getmanualsegmentdata():
     for file in filelist:
          print("Enter duration of file ", file, "or enter for no scaling")
          duration = input()
          if duration != "":
               duration = float(duration)
          else:
               duration = None

       # option to enter breakpoints,
       # like the identified timestamps for the six legs
       
          print("Enter breakpoints for file ", file, "as time from start to mark on the graph, separated by commas, or enter for none")
          breakpoints = input()
          if breakpoints != "":
               breakpoints = breakpoints.split(',')
               breakpoints = list(map(float, breakpoints))
               print(breakpoints, "entered as break points") 
          else:
               breakpoints = [0]
     return duration, breakpoints

##main
try:
     directory = sys.argv[1]
except:
     print("Enter directory to look for IMU csv files")
     directory = input()
     
filenamedictionarylist = []
for f in get_files(directory, 'csv'): # not recursive:
     filenamedictionarylist.append(f)

# This looks in the current directory for all leader files,
# and then filters based on criteria from user input.

print("Enter the criteria for filtering csv files as a substring of the filename.  If there are multiple sequential criteria for filtering, separate with a comma")
substrings = input()
substrings = substrings.split(",")
# print("Filtering on ", substrings)

 # initialize with the full list from the directory
filelist = filenamedictionarylist
for filtercriteria in substrings:
     substring = filtercriteria
     # print(filtercriteria, "is the current substring")
     filelist = [x for x in filelist if substring in x]
     print("files selected ", filelist)

criterianame = " and ".join(substrings)

# this takes the lists and plots all the instances
# of that card/group pairing  on a single plot
# This will be more useful if the CSV file has regularized timing
# (so ticks have some known correspondence to seconds)

dictlist = []
for file in filelist:
     try:
          breakpointsfile = sys.argv[2]                  
                    
     except:
          breakpointsfile = None

          
     csvdict = get_csv_data(file, breakpointsfile)
     csvdict = scalebysegment(csvdict) # includes breakpoints and duration
     dictlist.append(csvdict)
     
plotallthedata(dictlist, criterianame)
