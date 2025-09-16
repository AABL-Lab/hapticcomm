import matplotlib.pyplot as plt
import csv
import sys
import os
import glob
import math
import operator

def get_csv_data(filename):

     timestamp = []
     magnitude = []


     with open(filename, newline='') as csvfile:
          IMUreader = csv.reader(csvfile, delimiter=',', quotechar='|')
          #print(filename)
          count = 0
          # This skips the first row of the CSV file.
          next(IMUreader)
          mass = 1
          lasttimestamp = 0
          # now we turn the other rows into plottable lists
          for row in IMUreader:
               
               if count % 100 == 0: # and count < 3000:   #downsample
                    #timestamp.append(row[1].split(' ')[-1].replace('"',"")
                    timestamp.append(count)
                    # get magnitude of the accelleration
                    print(row[2], row[3], row[4])
                    magnitude.append(math.sqrt(float(row[2])**2+(float(row[3])-2)**2 + float(row[4])**2)/mass)
               count = count+1
               
              

          try:
               res = filename.split('/')
               #print("\n res", res)
               res2 = res[-1].split('.')
               res4 = res2[0].split('-')
               card = res4[0]
               role = res4[1]
               res3=res2[1].split('_')
               group = 'group'+res3[0].replace('bag','')
               print("group ", group, "card ", card, "role ", role)
          except:
               group = "unknown"
               card = "unknown"
               role = "unknown"
          
          filenamedictionary = {'name': group, 'card': card, 'role':role, 'time': timestamp, 'magnitude':magnitude}
          return filenamedictionary

def plotallthedata(rllist, rflist, plottitle):

     figure, axis = plt.subplots(2,1)

     figure.suptitle(plottitle)
     for set in rllist:
          axis[0].plot(set['time'],set['magnitude'], label=set['name'])
     axis[0].set_ylabel('Force Magnitude')
     axis[0].set_title('Robot Leader')
     #axis[0].set_ylim(0,5)
     #axis[0].set_xlim(0,80)
     axis[0].legend(loc=1)
     
     for set in rflist:
          axis[1].plot(set['time'],set['magnitude'], label=set['name'])
     axis[1].set_ylabel('Force Magnitude')
     axis[1].set_title('Robot Follower')
     #axis[1].set_ylim(0,5)
     axis[1].legend(loc=1)
     #axis[1].set_xlim(0,80)
     
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
                #print(file_path)
                res = file_path.split('/')
                global filename
                global subdir
                filename = res[-1]
                subdir = res[-2]
                #print(filename)
                yield file_path


def plot_by_card(dictlist, card):
     
     rllist = []
     rflist = []
   
     for d in dictlist:
          if d['card'] == card:
               if d['role'] == 'robotfollower':
                    rflist.append(d)
               elif d['role'] == 'robotleader':
                    rllist.append(d)
                    print("+++++")
          
     plotallthedata(rllist, rflist, card)


def average_magnitude(filenamedictionarylist):
     for group in range(1,17):
          cardlist = [x for x in filenamedictionarylist if 'group'+str(group)+'-' in x]

          rllist = [x for x in cardlist if 'robotleader' in x]
          rflist = [x for x in cardlist if 'humanleader' in x]
          hhlist = [x for x in cardlist if ('hh' in x) or ('humanhuman' in x)]
          
          list1 = []
          for file in rllist:
               csvdict = get_csv_data(file)
               avgmagnitude = sum(csvdict['magnitude'])/len(csvdict['magnitude'])
               list1.append(avgmagnitude)
               
          list2 = []
          for file in rflist:
               csvdict = get_csv_data(file)
               avgmagnitude = sum(csvdict['magnitude'])/len(csvdict['magnitude'])
               list2.append(avgmagnitude)


          
          list3 = []
          for file in hhlist:
               csvdict = get_csv_data(file)
               avgmagnitude = sum(csvdict['magnitude'])/len(csvdict['magnitude'])
               list3.append(avgmagnitude)
          #print(list1, list2, list3)
          with open('averagemagnitude.csv', mode='a') as data_file:
               data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
               data_writer.writerow(['group','condition', 'averagemagnitude'])
               for item in list1:
                    data_writer.writerow([group, 'rl',item])
               for item in list2:
                    data_writer.writerow([group,'rf',item])
               for item in list3:
                    data_writer.writerow([group, 'hh',item])
          with open ('average-average.csv', mode='a') as other_file:
               data_writer = csv.writer(other_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
               data_writer.writerow([group, 'rl', sum(list1)/len(list1)])
               data_writer.writerow([group, 'rf', sum(list2)/len(list2)])
               data_writer.writerow([group, 'hh', sum(list3)/len(list3)])
               
                                    
                    
          
##main
try:
     directory = sys.argv[1]
except:
     print("Enter directory to look for IMU csv files")
     directory = input()
filenamelist = []
dictlist = []
for f in get_files(directory, 'csv', recursive=True):
     filenamelist.append(f)
     r = get_csv_data(f)
     dictlist.append(r)

print("There are ", len(dictlist), "entries")
plot_by_card(dictlist, 'M')
plot_by_card(dictlist, 'jetski')
plot_by_card(dictlist, 'pentagon')
plot_by_card(dictlist, 'parasail')
plot_by_card(dictlist, 'dt')
plot_by_card(dictlist, 'beaker')

plt.ioff()
plt.show()
     

