import matplotlib.pyplot as plt
import csv
import sys
import os
import glob
import math
import operator
import matplotlib.patches as mpatches
import statistics



def get_csv_data(filename):

     timestamp = []
     magnitude = []


     with open(filename, newline='') as csvfile:
          IMUreader = csv.reader(csvfile, delimiter=',', quotechar='|')
          started = False
          countershift = 0
          print(filename)

          # This skips the first row of the CSV file.
          next(IMUreader)

          lasttimestamp = 0
          # now we turn the other rows into plottable lists
          for row in IMUreader:
               #print(row[0])
               if row[0].startswith('end'):
                    #print("Finished the set")
                    break
               elif row[0].startswith('(0'):
                    if started == False:
                         started = True
                    else:
                         countershift = lasttimestamp+1 # fix counter restarts
                         #print("counter offset is now ", countershift)
                         #print(row)
               try:
                    row[0] = row[0].replace('(', '')
                    row[6] = row[6].replace(')','')
                    
                    timestamp.append((float(row[0])+countershift)/3)
                    
                    #remove gravity except in glitches
                    if float(row[3])>1.0:
                         z_corr = (float(row[3])-9.8)
                    else:
                         z_corr = float(row[3])
                    
                    # get magnitude of the accelleration
                    magnitude.append(math.sqrt(float(row[1])**2+float(row[2])**2 + z_corr**2))
                    lasttimestamp = float(row[0])+countershift # for fixing counter restarts
               except Exception as e:
                    #print(e)
                    #print("Row ", row, "not added")
                    pass
          res = filename.split('/')
          group =res[-1].split('-')[0]
          condition = res[-1].split('-')[1]
          card = res[-1].split('-')[2].replace('.csv','')
          #print("group is ", group)
          
          filenamedictionary = {'name': group,'time': timestamp, 'magnitude':magnitude, 'card':card, 'condition':condition}
          return filenamedictionary

def plotallthedata(rllist, rflist, hhlist, plottitle):

     figure, axis = plt.subplots(3,1)

     figure.suptitle(plottitle)
     for set in rllist:
          axis[0].plot(set['time'],set['magnitude'], label=set['name'])
     axis[0].set_ylabel('Accel Magnitude')
     axis[0].set_title('Robot Leader')
     axis[0].set_ylim(0,5)
     axis[0].set_xlim(0,80)
     axis[0].legend(loc=1)
     
     for set in rflist:
          axis[1].plot(set['time'],set['magnitude'], label=set['name'])
     axis[1].set_ylabel('Accel Magnitude')
     axis[1].set_title('Robot Follower')
     axis[1].set_ylim(0,5)
     axis[1].legend(loc=1)
     axis[1].set_xlim(0,80)
     
     for set in hhlist:
          axis[2].plot(set['time'],set['magnitude'], label=set['name'])
     axis[2].set_ylabel('Accel Magnitude')
     axis[2].set_title('Human-Human')
     axis[2].set_ylim(0,5)
     axis[2].legend(loc=1)
     axis[2].set_xlim(0,80)
     # function to show the plot
     
     figure.set_label('Experiment Time')

     plt.tight_layout(pad=.2)

     # # Get handles and labels
     # handles, labels = figure.gca().get_legend_handles_labels()
     
     # # Sort the handles and labels based on labels
     # hl = sorted(zip(handles, labels), key=operator.itemgetter(1))
     # handles, labels = zip(*hl)
     
     # # Create the legend with the sorted handles and labels
     # figure.legend(handles, labels)
     plt.savefig(plottitle+".png",dpi=1000)
     plt.ion()

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


def plot_by_card(substring, altsubstring, filenamedictionarylist):
     cardlist = [x for x in filenamedictionarylist if (substring or altsubstring) in x]
#     print('\n\n Card list \n\n ', cardlist)

     rllist = [x for x in cardlist if 'robotleader' in x]
     rflist = [x for x in cardlist if 'humanleader' in x]
     hhlist = [x for x in cardlist if ('hh' in x) or ('humanhuman' in x)]

     list1 = []
     for file in rllist:
          csvdict = get_csv_data(file)
          list1.append(csvdict)
          
     list2 = []
     for file in rflist:
          csvdict = get_csv_data(file)
          list2.append(csvdict)
          
     list3 = []
     for file in hhlist:
          csvdict = get_csv_data(file)
          list3.append(csvdict)

     #cleanup to deal with files being named -M     
     try:
          substring.replace('-','')
     except:
          pass
     plotbycondition(list1, list2, list3, substring)

def plot_by_group(group, filenamedictionarylist):
     cardlist = [x for x in filenamedictionarylist if "group"+group+"-" in x]
     print('\n\n Card list \n\n ', cardlist, "for group", group)

     rllist = [x for x in cardlist if 'robotleader' in x]
     rflist = [x for x in cardlist if 'humanleader' in x]
     hhlist = [x for x in cardlist if ('hh' in x) or ('humanhuman' in x)]

     list1 = []
     for file in rllist:
          csvdict = get_csv_data(file)
          list1.append(csvdict)
          
     list2 = []
     for file in rflist:
          csvdict = get_csv_data(file)
          list2.append(csvdict)
          
     list3 = []
     for file in hhlist:
          csvdict = get_csv_data(file)
          list3.append(csvdict)

     #cleanup to deal with files being named -M     
     try:
          substring.replace('-','')
     except:
          pass
     plotbycondition(list1, list2, list3, group, multiplots=True)

def plotbycondition(rllist, rflist, hhlist, plottitle, multiplots=False):
     if multiplots:
          figure, axis = plt.subplots(3,1)
          
          for set in rllist:
               axis[0].plot(set['time'],set['magnitude'], label='leader', color=(94/255, 201/255, 98/255))
               
          for set in rflist:
               axis[1].plot(set['time'],set['magnitude'], label='follower',color=(68/255, 1/255, 84/255))
                    
          for set in hhlist:
               axis[2].plot(set['time'],set['magnitude'], label='follower',color=(253/255, 231/255, 37/255))
     
     else:
          figure = plt.figure()
          for set in rllist:
               plt.plot(set['time'],set['magnitude'], label='leader', color=(94/255, 201/255, 98/255))
     
          for set in rflist:
               plt.plot(set['time'],set['magnitude'], label='follower',color=(68/255, 1/255, 84/255))

          for set in hhlist:
               plt.plot(set['time'],set['magnitude'], label='follower',color=(253/255, 231/255, 37/255))



     #plt.suptitle(plottitle)
     
     plt.ylabel('Acceleration Magnitude')
     leader = mpatches.Patch(color=(94/255, 201/255, 98/255), label='Robot Leaders')
     follower = mpatches.Patch(color=(68/255, 1/255, 84/255), label='Robot Followers')
     human  = mpatches.Patch(color=(253/255, 231/255, 37/255), label='Human-Human')     
     plt.legend(handles=[leader, follower,human])
     #plt.ylim(0, 6)
     plt.xlabel('Time in seconds')

     
          

     
     figure.set_label('Experiment Time')

     #plt.tight_layout(pad=.2)

     # # Get handles and labels
     # handles, labels = figure.gca().get_legend_handles_labels()
     
     # # Sort the handles and labels based on labels
     # hl = sorted(zip(handles, labels), key=operator.itemgetter(1))
     # handles, labels = zip(*hl)
     
     # # Create the legend with the sorted handles and labels
     # figure.legend(handles, labels)
     plt.savefig(plottitle+"_IMU.png",dpi=1000)
     plt.ion()




def plotjustone(filename):
     set = get_csv_data(filename)

     print("graphing ", set['name'])
     figure = plt.figure()
     plt.suptitle(set['name']+set['card']+set['condition'])
     plt.plot(set['time'],set['magnitude'], label='leader', color=(68/255, 1/255, 84/255))
     plt.ion


def plotmultiple(filenames):
     colors = [ (253, 231, 37)/255,
                (94, 201, 98)/255,
                (33, 145, 140)/255,
                (59, 82, 139)/255,
                (68, 1, 84)/255,
                ]
     
     for n in range(len(filenames)):
          set = get_csv_data(filename[n])
          
          print("graphing ", set['name'])
          figure = plt.figure()
          plt.suptitle(set['name']+set['card']+set['condition'])
          plt.plot(set['time'],set['magnitude'], label='leader', color=colors[n])
          plt.ion


     
     
##main
try:
     directory = sys.argv[1]
except:
     print("Enter directory to look for IMU csv files")
     directory = input()
filenamedictionarylist = []
for f in get_files(directory, 'csv'): # not recursive:
     filenamedictionarylist.append(f)
   #  plotjustone(f)

#plot_by_card('dt', 'DT', filenamedictionarylist)
#plot_by_card('beaker','Beaker', filenamedictionarylist)
#plot_by_card('-M','-M', filenamedictionarylist)
#plot_by_card('parasail','Parasail', filenamedictionarylist)
#plot_by_card('pentagon', 'Pentagon',filenamedictionarylist)
#plot_by_card('jetski', 'Jetski', filenamedictionarylist)

for group in range(1,18):
    plot_by_group(str(group),filenamedictionarylist) 




plt.ioff()
plt.show()
