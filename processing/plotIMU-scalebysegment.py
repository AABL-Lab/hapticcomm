import matplotlib.pyplot as plt
import csv
import sys
import os
import glob


def get_csv_data(filename, duration=None):
# if duration is provided, this will scale the rowcount
# by the number of seconds provided
     timestamp = []
     Xaccel = []
     Yaccel = []
     Zaccel = []
     Xrotation = []
     Yrotation = []
     Zrotation = []


     with open(filename, newline='') as csvfile:
          IMUreader = csv.reader(csvfile, delimiter=',', quotechar='|')
          started = False
          countershift = 0

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
                   

          # print("time:", timestamp)
          # print("Xaccel = ", Xaccel)
          # print("Yaccel = ", Yaccel)
          # print("Zaccel = ", Zaccel)
          # print("Xrot = ", Xrotation)
          # print("Yrot = ", Yrotation)
          # print("Zrotation = ", Zrotation)
          
          res = filename.split('/')
          group =res[-1].split('-')[0]
          subgroup = res[-1].split('-')[1]
          print("group is ", group)



          
          filenamedictionary = {'name': subgroup,'time': timestamp, 'Xaccel':Xaccel, 'Yaccel':Yaccel, 'Zaccel':Zaccel, 'Xrotation':Xrotation, 'Yrotation':Yrotation, 'Zrotation':Zrotation}

          ######Correcting Timestamps with Duration ####
          if duration != None:
               newtimestamp = []
               print("Correcting timestamps for provided duration ", duration, " seconds")
               for time in timestamp:
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

def scalebysegment(filenamedictionarylist, breakpoints, duration):
     
     newtimestamp = []
     print("Breaking segments into ", breakpoints, "segments and scaling as if each segment were 10 time units")
     timestamps = filenamedictionarylist["time"]
     lastpoint = 0
     sectionstart = 0
     
     for time in timestamps:
          for point in breakpoints:
               if time < point and time >= lastpoint:
                    newtime = time/point*10 + sectionstart
                    print("In segment less than breakpoint ", point, "the time was ", time, "and the new time is ", newtime)
                    newtimestamp.append(time)
               else:
                    lastpoint = point # advance to the next point
                    sectionstart = sectionstart+10
          lastpoint = 0
          sectionstart = 0

     print("new times :", newtimestamp)
     filenamedictionarylist['time'] = newtimestamp
     return filenamedictionarylist


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
     csvdict = get_csv_data(file, duration)
     csvdict['breakpoints'] = breakpoints
     #csvdict = scalebysegment(csvdict, breakpoints, duration)
     dictlist.append(csvdict)
     
plotallthedata(dictlist, criterianame)
