import matplotlib.pyplot as plt
import csv


timestamp = []
Xaccel = []
Yaccel = []
Zaccel = []
Xrotation = []
Yrotation = []
Zrotation = []


with open('IMU_readings2Aug2022.csv', newline='') as csvfile:
     IMUreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     
     # This skips the first row of the CSV file.
     next(IMUreader)

     # now we turn the other rows into plottable lists
     for row in IMUreader:
          print(row[0])
          if row[0].startswith('Current'):
               break
               print("Finished the first set")
          
          #print(row)
          timestamp.append(row[0])
          Xaccel.append(row[1])
          Yaccel.append(row[2])
          Zaccel.append(row[3])
          Xrotation.append(row[4])
          Yrotation.append(row[5])
          Zrotation.append(row[6])

  
# x-axis label
plt.xlabel('Experiment Time')

# Define all the subplots 
#plt.subplot(2,3,1) # 2 rows, 3 columns, this is the 1st plot)
fig1 = plt.figure("X Acceleration")
plt.plot(timestamp,Xaccel)
# frequency label
plt.ylabel('X Acceleration')
# every_nth = 10
# for n, label in enumerate(plt.xaxis.get_ticklabels()):
#     if n % every_nth != 0:
#         label.set_visible(False)

# for n, label in enumerate(plt.yaxis.get_ticklabels()):
#     if n % every_nth != 0:
#         label.set_visible(False)


#plt.subplot(2,3,2)
fig2 = plt.figure("Y Acceleration")
plt.plot(timestamp, Yaccel)
# frequency label
plt.ylabel('Y Acceleration')

#plt.subplot(2,3,3)
fig3 = plt.figure("Z Acceleration")
plt.plot(timestamp, Zaccel)
plt.show
# frequency label
plt.ylabel('Z Acceleration')

#plt.subplot(2,3,4)
fig4 = plt.figure("X Rotation")
plt.plot(timestamp, Xrotation)

#plt.subplot(2,3,5)
fig5 = plt.figure("Y Rotation")
plt.plot(timestamp, Yrotation)
#plt.subplot(2,3,6)
fig6 = plt.figure("Z Rotation")
plt.plot(timestamp, Zrotation)


# showing legend
#plt.legend()



#plt.plot(x,y, marker = 'o')

# function to show the plot
plt.show()
