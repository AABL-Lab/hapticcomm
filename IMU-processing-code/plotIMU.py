import matplotlib.pyplot as plt
import csv


timestamp = []
Xaccel = []
Yaccel = []
Zaccel = []
Xrotation = []
Yrotation = []
Zrotation = []

print("Enter filename (with .csv) to process")
filename = input()


with open(filename, newline='') as csvfile:
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
          try:
               row[0] = row[0].replace('(', '')
               row[6] = row[6].replace(')','')

               timestamp.append(float(row[0]))
               Xaccel.append(float(row[1]))
               Yaccel.append(float(row[2]))
               Zaccel.append(float(row[3]))
               Xrotation.append(float(row[4]))
               Yrotation.append(float(row[5]))
               Zrotation.append(float(row[6]))
          except:
               print("Row ", row, "not added")
               pass
print("Xaccel = ", Xaccel)
print("Yaccel = ", Yaccel)
print("Zaccel = ", Zaccel)
print("Xrot = ", Xrotation)
print("Yrot = ", Yrotation)
print("Zrotation = ", Zrotation)
          
  
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
