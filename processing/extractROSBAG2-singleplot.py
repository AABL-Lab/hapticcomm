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
          #print(filename)
          count = 0
          # This skips the first row of the CSV file.
          next(IMUreader)
          mass = 1
          lasttimestamp = 0
          # now we turn the other rows into plottable lists
          for row in IMUreader:
               if count ==0:
                    starttime = row[1].split(' ')[-1].replace('"',"")
               if count % 100 == 0 and count < 3200:   #downsample
                    
                    # this would make a prettier x axis, but need to convert strings to real times to subtract
                    #timestamp.append(row[1].split(' ')[-1].replace('"',"")-starttime)

                    # kind of a hack for ~ 100hz data
                    timestamp.append(count/100)
                    
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

     figure = plt.figure()

     #plt.suptitle(plottitle)
     for set in rllist:
          plt.plot(set['time'],set['magnitude'], label='leader', color=(94/255, 201/255, 98/255))
     plt.ylabel('Force Magnitude')
     leader = mpatches.Patch(color=(94/255, 201/255, 98/255), label='Robot Leaders')
     follower = mpatches.Patch(color=(68/255, 1/255, 84/255), label='Robot Followers')
     plt.legend(handles=[leader, follower])
     plt.ylim(0, 40)
     plt.xlabel('Time in seconds')
     
     for set in rflist:
          plt.plot(set['time'],set['magnitude'], label='follower',color=(68/255, 1/255, 84/255))
          

     
     figure.set_label('Experiment Time')

     #plt.tight_layout(pad=.2)

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
     print(filenamedictionarylist)

     for group in range(1,18):
          cardlist = [x for x in filenamedictionarylist if 'group'+str(group) == x['name']]
          rllist = [x for x in cardlist if 'robotleader' in x['role']]
          rflist = [x for x in cardlist if 'robotfollower' in x['role']]


          rldlist = []
          for csvdict in rllist:
               avgmagnitude = sum(csvdict['magnitude'])/len(csvdict['magnitude'])
               rld = {}
               rld['list']=avgmagnitude
               stddev = statistics.stdev(csvdict['magnitude'])
               rld['stddev']=stddev
               rld['card']=csvdict['card']
               
               rldlist.append(rld)

          rfdlist = []
          for csvdict in rflist:
               avgmagnitude = sum(csvdict['magnitude'])/len(csvdict['magnitude'])
               rfd = {}
               rfd['list']=avgmagnitude
               stddev = statistics.stdev(csvdict['magnitude'])
               rfd['stddev']=stddev
               rfd['card']=csvdict['card']
               rfdlist.append(rfd)

          print(rfdlist)


          with open('averagemagnitude.csv', mode='a') as data_file:
               data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
               data_writer.writerow(['group','condition','card', 'averagemagnitude', 'stdev'])
               for item in rldlist:
                    data_writer.writerow([group, 'rl', item['card'],item['list'],item['stddev']])
                    
               for item in rfdlist:
                    data_writer.writerow([group,'rf',item['card'],item['list'],item['stddev']])
                       
          with open ('average-average.csv', mode='a') as other_file:
               data_writer = csv.writer(other_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
               data_writer.writerow(['group', 'condition', 'averagemagnitude', 'stddev'])

               listofmags = []
               for item in rldlist:
                    listofmags.append(item['list'])
               rlaverageaverage = statistics.mean(listofmags)
               rlavgstd = statistics.stdev(listofmags)
               
               data_writer.writerow([group, 'rl', rlaverageaverage, rlavgstd])
               
               
               listofmags = []
               for item in rfdlist:
                    listofmags.append(item['list'])
               rfaverageaverage = statistics.mean(listofmags)
               rfavgstd = statistics.stdev(listofmags)

               data_writer.writerow([group, 'rf', rfaverageaverage, rlavgstd])

                                    
                    
          
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
average_magnitude(dictlist)
plt.ioff()
plt.show()
     

