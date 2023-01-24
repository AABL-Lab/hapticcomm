import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

waypoints = pd.read_csv('waypoints.csv', header = 'infer')

# check that it loaded properly
print ("Head",  waypoints.head()) 
print("Columns", waypoints.columns)
positionnames = waypoints.filter(items=['positionname'])
print("position names", positionnames)
print(positionnames.shape)
print("position 4", positionnames.iloc[4]) 