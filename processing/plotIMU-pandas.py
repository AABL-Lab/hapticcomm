import matplotlib.pyplot as plt
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
import numpy as np
import glob
import os
import sys
from datetime import datetime, timedelta, date, time
import itertools

import re

def get_csv_data(filename, breakpointfilename=None, option=None, rezero=False):
    print("#############Processing File ", filename, "#######################")
    # find the beginning of the actual IMU data and strip off the start time, removing any prior lines
    with open(filename, 'r') as f:
        first_line = f.readline().strip()
        while first_line.split(',')[0][:3]!="UTC":
            first_line = f.readline().strip() # strip off the next line
        
    # Extract clocktimeIMUstart as a datetime object from that first line
    print(first_line, "is the first line of the IMU csv")
    clocktimeIMUstart_str = first_line.split(',')[3]
    match = re.search(r'(\d{1,2}):(\d{1,2}):(\d{1,2})', clocktimeIMUstart_str)
    if match:
        clocktimeIMUstart = datetime.strptime(match.group(), "%H:%M:%S").time()
    
    
    df = pd.read_csv(filename, skiprows=1, names=["timestamp", "X Acceleration", "Y Acceleration", "Z Acceleration", "X Rotation", "Y Rotation", "Z Rotation"], engine='python')
    df = df.applymap(lambda x: str(x).replace('(', '').replace(')', ''))


    # Identify rows that have errors in conversion (but do not drop them yet)
    error_rows = df.apply(lambda row: any("error" in str(x).lower() for x in row), axis=1)
    # Convert to numeric and coerce errors (errors will be NaN)
    df = df.apply(pd.to_numeric, errors='coerce')
    # Drop rows that contain NaN values (this includes the original error rows)
    df = df.dropna()

    df.reset_index(drop=True, inplace=True)

    # Renumber the IMU datapoints so that they are continuous
    df['timestamp'] = range(1, len(df) + 1)


    # pull out labels for this data from the IMU filename before we add it to our list of dataframes to plot
    metadata = filename.split('/')[-1].split('-')
    group, subgroup, card = metadata[0].replace('group', ''), metadata[1], metadata[2].split('.')[0]
    #print(f"Group: {group}, Subgroup: {subgroup}, Card: {card}")




    ## If breakpoints were provided in a file, we will pull them and some other timing data from there
    if breakpointfilename:
        breakpoint_df = pd.read_csv(breakpointfilename)
        #print(f"Attempting to read breakpoint file: {breakpointfilename}")
        #print(f"Breakpoints DataFrame:\n{breakpoint_df.head()}")
        
        breakpoint_df = breakpoint_df.applymap(lambda x: str(x).replace('(', '').replace(')', ''))
        #print(f"Unique Groups: {breakpoint_df['Group'].unique()}")
        #print(f"Unique Subgroups: {breakpoint_df['Subgroup'].unique()}")
        #print(f"Unique CARDs: {breakpoint_df['CARD'].unique()}")
        

        matched_bp = breakpoint_df[
            (breakpoint_df['Group'].astype(str).str.strip() == str(group).strip()) &
            (breakpoint_df['Subgroup'].astype(str).str.strip() == subgroup.strip()) &
            (breakpoint_df['CARD'].astype(str).str.strip() == card.strip())
        ]
        firstmatch = matched_bp.iloc[0]
        breakpoints = firstmatch[['Start', 'L1E', "L2E", "L3E", "L4E", "L5E", 'Duration']].values.flatten().tolist()
        # This will flip y-axis data if the leader was seated on the door side
        try:
            leaderside = int(float(firstmatch['Leaderside']))
            if leaderside == 2:
                flip=True
            else:
                flip=False
            print("Leader side is ", leaderside, "so flip is", flip)
        except:
            print("Leaderside not found")
            flip=False
 

        print("Breakpoints after firstmatch are ", breakpoints)
        

        breakpoints = [float(bp) for bp in breakpoints]
        duration = breakpoints[-1] if breakpoints else df['timestamp'].iloc[-1]

        index_list = matched_bp.index.tolist()
        #print("Index of matched_bp", index_list)
        trial_starttime = str(matched_bp.loc[index_list[0],'24hStart'])
        trial_starttime  = trial_starttime.replace(" ", "")  # Remove all spaces
        #print("Trial Start Time ", trial_starttime)

        #### We want to exclude datasets that do not have breakpoints in the file, like the ones with no video data
        if trial_starttime == "exclude":
            return # this will return the function as None
                   #and then get excluded from the printing
        else:
            trial_starttime = datetime.strptime(trial_starttime, "%H:%M:%S").time()

            


        print("Trial duration is ", duration)
        print("Breakpoints are ", breakpoints)


        
    else:
        duration, breakpoints, trial_starttime = get_manual_segment_data()

    waittime = datetime.combine(date.min,trial_starttime)-  datetime.combine(date.min, clocktimeIMUstart)
    #print("difference between trial start time", trial_starttime, " and IMU clock start time ", clocktimeIMUstart," is ", waittime)



    if option == "clocktime":
        
        df, scaledbreakpoints = rescale_all_data_clocktime(df, breakpoints, waittime, filename, clocktimeIMUstart, duration)
    
    elif option == "segment":
        df, scaledbreakpoints = rescale_by_segment(df, breakpoints, waittime, filename)
    elif option == "100":
        df, scaledbreakpoints = rescale_alldata_1to100(df, breakpoints, waittime, filename)

        
    # rezero the data based on the average acceleration in the first half of the wait time
    if rezero == True:
        df = rezero_IMU(df, waittime)


    # flip y axis data if so that x+ is always towards the leader
    if flip==True:
        df['X Acceleration'] = -df['X Acceleration']
        df['X Rotation'] = -df['X Rotation']
        print("X axis flipped")
    


        
    return {'name': filename, 'data': df, 'breakpoints': scaledbreakpoints, 'duration': duration, 'clocktimeIMUstabyrt': clocktimeIMUstart}


def rezero_IMU(df, waittime):
    half_wait_time = waittime.total_seconds() / 2

    # Initialize a new DataFrame to store the adjusted data
    adjusted_df = df.copy()
    
    # Loop through each column that represents an axis and adjust it
    axis_columns = ["X Acceleration", "Y Acceleration", "Z Acceleration", "X Rotation", "Y Rotation", "Z Rotation"]


    # Find the index where the timestamp <= half the wait time
    #print("the type of wait_time is ", type(half_wait_time))
    
    half_wait_index = df[df["time"] <= half_wait_time].index
    
    # For each axis, subtract the mean of the first half of the wait time
    for axis in axis_columns:
        # Get the mean of the first half of the data
        mean_value = df.loc[half_wait_index, axis].mean()
        
        # Subtract this mean from the entire axis data
        adjusted_df[axis] = df[axis] - mean_value

        # Check if the adjusted values are more than +/- 3 from the mean and set those values to 0
        adjusted_df[axis] = adjusted_df[axis].apply(lambda x: 0 if abs(x) > 3 else x)
    

    return adjusted_df



def rescale_by_segment(df, breakpoints, waittime, filename):
    # for each segment, we want to rescale first 1 to 100 (to deal with the time being in seconds and the data not)
    # and then by breakpoint so that we can try to even the breakpoints out.
    df, breakpoints = rescale_alldata_1to100(df, breakpoints, waittime, filename)


    scaled_segments = []
    
    # Process the first segment (before the first breakpoint, negative time)
    first_segment = df[df['time'] < breakpoints[0]].copy()
    first_segment.loc[:, 'segmentscaledtime'] =  np.interp(first_segment['time'], (df['time'].min(), breakpoints[0]), (-1,0))
    scaled_segments.append(first_segment)
    #print("first segment is ", first_segment)
    
    # Process all intermediate segments
    print(breakpoints, len(breakpoints), "length and quantity of breakpoints")
    for i in range(len(breakpoints)-1): # for all but the last breakpoint
        lower, upper = breakpoints[i], breakpoints[i+1]
        segment = df[(df['time'] >= lower) & (df['time'] < upper)].copy()  # Explicit copy
        if not segment.empty:
            segment.loc[:, 'segmentscaledtime'] = np.interp(segment['time'], (lower, upper), (i , (i + 1) ))
            scaled_segments.append(segment)


    ######This is never needed because we do not have a clock endtime for the file, and are
    ######assuming that clock-endtime from video  == file-endtime, so the scaled data always comes
    ###### into this file with 100 as the timestamp for the last datapoint. 

    # Process the last segment (after the last breakpoint) if any
    last_segment = df[df['time'] > breakpoints[-1]].copy()
    last_segment.loc[:, 'segmentscaledtime'] = np.interp(last_segment['time'], (breakpoints[-1], df['time'].max()), (len(breakpoints), (len(breakpoints)) ))
    scaled_segments.append(last_segment)

    
    # Combine all segments into a new DataFrame
    df_scaled = pd.concat(scaled_segments).sort_values('time')
    
    # Reset index
    df_scaled.reset_index(drop=True, inplace=True)
    #print(df_scaled['segmentscaledtime'])
    
    return df_scaled, [-1,0,1,2,3,4,5,6]
        

def rescale_all_data_clocktime(df, breakpoints, waittime, filename, clocktimeIMUstart, task_duration):
    # For this we re-scale the IMU data so it all corresponds to clock time even when the IMU data rate
    # varies between files
    #print("type of clocktime, waittime, task_duration", type(clocktimeIMUstart),type(waittime), type(task_duration))
    base_date = datetime(1970, 1, 1)  # Dummy date (1970-01-01) for conversion
    clocktimeIMUstart_dt = base_date.replace(hour=clocktimeIMUstart.hour,
                                             minute=clocktimeIMUstart.minute,
                                             second=clocktimeIMUstart.second)
    
    # Step 2: Add `waittime` and `task_duration` (convert task_duration to timedelta)
    time_end = clocktimeIMUstart_dt + waittime + timedelta(seconds=task_duration)

    #print("type of clocktime, waittime, task_duration", type(clocktimeIMUstart),type(waittime), type(task_duration)) 

    time_start = clocktimeIMUstart
    #time_end = clocktimeIMUstart + waittime + timedelta(seconds= task_duration)

    #print("start time is ", time_start , "trial end time is ", time_end)

    # Compute the time elapsed in seconds from the start time for each row
    total_timestamps = df['timestamp'].iloc[-1] # last time in the file
    df['time'] = (df['timestamp'] / total_timestamps) * task_duration
    
    # Adjust the breakpoints to be in seconds
    if 0 <= waittime.total_seconds() <= 15:
        bp_start = breakpoints[0]
        bp_end = breakpoints[-1] + waittime.total_seconds()
        
        # Add the wait time to each breakpoint to adjust them to actual seconds
        print("Breakpoints start at ", bp_start, "and end at ", bp_end)
        scaledbreakpoints = [
            (bp + waittime.total_seconds())  # Add the wait time to each breakpoint
            for bp in breakpoints
        ]
    else:
        print("Wait time for file ", filename , "may be miscalculated as it is more than 15 seconds, check data")
        
    return df, scaledbreakpoints
    
    
    
        
def rescale_alldata_1to100(df, breakpoints, waittime, filename):
    # We want all the IMU files to display on the same scale, so we rescale the timestamps over 100 units
    time_start = df['timestamp'].iloc[0]
    time_end = df['timestamp'].iloc[-1]

    # leaving the timestamps aside, we make a new variable called time that ranges from 0-100 and rescale the data within that
    df['time'] = round(((df['timestamp'] - time_start) / (time_end - time_start) * 100))

    # the breakpoints do not include the wait time, so we need to add that to account for the dead time at the start of the trial
    if 0 <= waittime.total_seconds() <= 15:
        
        bp_start = breakpoints[0]
        bp_end = breakpoints[-1]+waittime.total_seconds()
        # add the wait time to each breakpoint, then scale them out of 100
        #print("Breakpoints start at ", bp_start, "and end at ", bp_end)
        scaledbreakpoints = [waittime.total_seconds()]
        for bp in breakpoints:
            scaledbreakpoints.append(round(((bp + waittime.total_seconds()) / bp_end) * 100))
       
        
        print("scaled breakpoints are", scaledbreakpoints)
    else:
        print("Wait time for file ", filename , "may be miscalculated as it is more than 15 seconds, check data")
        scaledbreakpoints = [
            ((bp) / breakpoints[-1]) * 100
            for bp in breakpoints
        ]
        input()

          
    return df, scaledbreakpoints

     



def get_manual_segment_data():
    duration = float(input("Enter file duration or leave blank for none: ") or 0)
    breakpoints = input("Enter breakpoints separated by commas or leave blank: ")
    breakpoints = list(map(float, breakpoints.split(','))) if breakpoints else []
    trial_starttime = input("Enter trial start time as HH:MM:SS")
    trial_starttime = datetime.strptime(trial_starttime, "%H:%M:%S").time()

    ##fixme: this will need to also collect file and trial start time offsets if we want to use it to rescale
    return duration, breakpoints, trial_starttime


def plot_data(filenamedictionary_list, title,plotselect=['X Acceleration', 'Y Acceleration', 'Z Acceleration', 'X Rotation', 'Y Rotation', 'Z Rotation'], options="segment",save=False):

            
    try: # if the time has been scaled by segment, put in the segment labels
        filenamedictionary_list[0]['data']['segmentscaledtime']
        # X positions for labels
        divider_positions = [-1, 0, 1, 2, 3, 4, 5, 6]
        dividerlabels = ["recording start", "trial start", "start leg 2", 
                         "start leg 3", "start leg 4", "start leg 5","start leg 6",  "end trial"]
    except KeyError:
        divider_positions = []
        dividerlabels = []
    
    # Create a figure with subplots for each axis
    fig, axes = plt.subplots(len(plotselect), 1, sharex=True, figsize=(10, 8))
    fig.suptitle(title)
    labels = plotselect

    # If you want randomly cycled colors (like if you are plotting lots of lines)
    #color_cycle = itertools.cycle(['r', 'g', 'b', 'c', 'm', 'y', 'k'])  # You can add more colors to the list
    

    # Loop through each dataset in the list
    for dataset in filenamedictionary_list:

        # Get a color for the current dataset from the color cycle
        #color = next(color_cycle)

        # set color based on interaction type
        if "hh" in dataset['name']:
            color="r"
        elif "robotleader" in dataset['name']:
            color="b"
        elif "humanleader" in dataset["name"]:
            color="g"
        else:
            color="black"
        
        #print("the dataset is ", dataset["name"])
        # Loop through each label
        for i, dlabel in enumerate(labels):
            # Plot the dataset
            try:
                #print(i, dlabel)
                #print(dataset['data'][dlabel])
                #input()
                #print(dataset['data'][dlabel[:1]])
                print(dataset['data']['segmentscaledtime'])
                axes[i].plot(dataset['data']['segmentscaledtime'], dataset['data'][dlabel], label=dataset['name'], color=color,linestyle="-",marker=".")
                if "Rot" in dlabel:
                    axes[i].set_ylim(-0.5,0.5)
                else:
                    axes[i].set_ylim(-3,3)
                
            except TypeError: # if we only have one plot
                axes.plot(dataset['data']['segmentscaledtime'], dataset['data'][dlabel], label=dataset['name'], color=color,linestyle="-",marker=".")
                axes.set_ylim(-3,3)
            except KeyError:
                axes[i].plot(dataset['data']['time'], dataset['data'][dlabel], label=dataset['name'], color=color,linestyle="-",marker=".")
                axes[i].set_ylim(-3,3)
            # Add vertical lines to all plots
            for pos, slabel in zip(divider_positions, dividerlabels):
                try:
                    axes[i].axvline(x=pos, color='gray', linestyle='--', alpha=0.7)  # Add vertical line
                except TypeError:
                    axes.axvline(x=pos, color='gray', linestyle='--', alpha=0.7)  # Add vertical line
            # Set the ylabel for the current axis
            try:
                if "Rot" in dlabel:
                    axes[i].set_ylabel(dlabel.replace("ation",".(rad/s)"))
                elif "Acc" in dlabel:
                    axes[i].set_ylabel(dlabel.replace("eleration",".(g)"))
            except TypeError:
                axes.set_ylabel(dlabel)
                
    # Set common x-label and ticks
    try:
        axes[-1].set_xlabel(options)
    except TypeError:
        axes.set_xlabel(options)
    
    # if we are using the segment data, label the dividers just at the top
    for pos, slabel in zip(divider_positions, dividerlabels):
        # Place label just above the x-data plot with diagonal rotation
        try:
            axes[0].text(pos, axes[0].get_ylim()[1], slabel, 
                      rotation=45, ha='left', va='bottom', fontsize=10, color='black')
        except TypeError:
            axes.text(pos, axes.get_ylim()[1], slabel, 
                      rotation=45, ha='left', va='bottom', fontsize=10, color='black')
            
    # Extract min and max values from the list of dictionaries
    try:
        min_time = int(min(d['data']['segmentscaledtime'].min() for d in filenamedictionary_list))
        max_time = int(max(d['data']['segmentscaledtime'].max() for d in filenamedictionary_list))
        #for d in filenamedictionary_list:
        #    print(d['data']['segmentscaledtime'])
 
    except KeyError:
        min_time = int(min(d['data']['time'].min() for d in filenamedictionary_list))
        max_time = int(max(d['data']['time'].max() for d in filenamedictionary_list))

    #print("min time is ", min_time, "max time is ", max_time)
    
    # Set the x-axis limits to match the data range
    try:
        axes[-1].set_xlim(min_time-.5, max_time+1)
    
        # Set the x-axis ticks properly
        axes[-1].set_xticks(range(min_time, max_time, int(round((max_time-min_time)/10))))
    except TypeError:
        axes.set_xlim(min_time-.5, max_time)
    
        # Set the x-axis ticks properly
        axes.set_xticks(range(min_time, max_time, int(round((max_time-min_time)/10))))
        
        
    # Add legend
    plt.legend(loc='best',  fontsize='small', bbox_to_anchor=(1.1, 0))
    
    # Adjust layout to avoid overlap
    plt.tight_layout()
    if save==True:
        plt.savefig('graphs/'+title+'.png')
        plt.ion()
    plt.show()
 
def plot_mean_shaded(filenamedictionary_list, title ,plotselect=['X Acceleration', 'Y Acceleration', 'Z Acceleration'], options="segment",save=True):
    divider_positions = [-1, 0, 1, 2, 3, 4, 5, 6]
    dividerlabels = ["recording start", "trial start", "start leg 2", 
                         "start leg 3", "start leg 4", "start leg 5","start leg 6",  "end trial"]
    #plot mean with std for each axis
    
    # Set the font size to match IEEE standard
    plt.rcParams.update({'font.size': 8, 'axes.labelsize': 8, 'legend.fontsize': 8, 'xtick.labelsize': 8, 'ytick.labelsize': 8})

    x_sets = []
    y_sets = []
    z_sets = []
    for dataset in filenamedictionary_list:
        x_sets.append(dataset['data']['X Acceleration'])
        print("length ", len(dataset['data']['X Acceleration']),"x data from ", dataset['name'],"added to be averaged")
        y_sets.append(dataset['data']['Y Acceleration'])
        z_sets.append(dataset['data']['Z Acceleration'])
        
    mean_x = np.mean(x_sets, axis=0)
    std_x = np.std(x_sets, axis=0)
    mean_y = np.mean(y_sets, axis=0)
    std_y = np.std(y_sets, axis=0)
    mean_z = np.mean(z_sets, axis=0)
    std_z = np.std(z_sets, axis=0)
        
 
    fig, axes = plt.subplots()
    axes.plot(dataset['data']['segmentscaledtime'],mean_x)
    axes.fill_between(range(len(mean_x)), mean_x-std_x, mean_x+std_x, alpha=0.2)
    axes.plot(dataset['data']['segmentscaledtime'],mean_y)
    axes.fill_between(range(len(mean_y)), mean_y-std_y, mean_y+std_y, alpha=0.2)
    axes.plot(dataset['data']['segmentscaledtime'],mean_z)
    axes.fill_between(range(len(mean_z)), mean_z-std_z, mean_z+std_z, alpha=0.2)

   # if we are using the segment data, label the dividers just at the top
    for pos, slabel in zip(divider_positions, dividerlabels):
        # Place label just above the x-data plot with diagonal rotation
        try:
            axes.text(pos, axes.get_ylim()[1], slabel, 
                      rotation=45, ha='left', va='bottom', fontsize=10, color='black')
        except TypeError:
            axes.text(pos, axes.get_ylim()[1], slabel, 
                      rotation=45, ha='left', va='bottom', fontsize=10, color='black')

    # Add vertical lines to all plots
    for pos, slabel in zip(divider_positions, dividerlabels):
        try:
            axes.axvline(x=pos, color='gray', linestyle='--', alpha=0.7)  # Add vertical line
        except TypeError:
            axes.axvline(x=pos, color='gray', linestyle='--', alpha=0.7)  # Add vertical line
            
            
    axes.set_xlim(-1,6)
    axes.set_ylim(-3,3)
    plt.show()

    

    
def get_files(directory, extension):
    return glob.glob(os.path.join(directory, f'*.{extension}'))


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else input("Enter directory to look for CSV files: ")
    files = get_files(directory, 'csv')
    substrings = input("Enter filename filter criteria as a|b,c,d|e : ").split(',')
    # Parse the substrings into logical groups
    filter_groups = [group.split('|') for group in substrings]
    
    # Filter the files based on the parsed filter groups
    files = [f for f in files if all(any(sub in f for sub in group) for group in filter_groups)]

    
    print("Files found are ", files)
    
    breakpoints_file = sys.argv[2] if len(sys.argv) > 2 else None
    datasets = []

    options = input("Which time scaling option do you want? clocktime, segment, 100 (enter to default to  segment)")
    if options =="":
        options="segment"
        
    rezero_option = input("Do you want to rezero the acceleration data? y or n (or enter for y)")
    if rezero_option =='n' or rezero_option == 'N':
        rezero = False
    else:
        rezero = True

    plotoption = input("Which plots do you want to show?  all, linear, rotational, xlin,ylin,zlin,xrot,yrot,zrot? (enter for default all)")
    if plotoption =="all" or plotoption =="":
        plotselect = ['X Acceleration', 'Y Acceleration', 'Z Acceleration', 'X Rotation', 'Y Rotation', 'Z Rotation']
    elif plotoption=="linear":
        plotselect= ['X Acceleration', 'Y Acceleration', 'Z Acceleration']
    elif plotoption=="rotational":
        plotselect=['X Rotation', 'Y Rotation', 'Z Rotation']
    elif plotoption=="xlin":
        plotselect=['X Acceleration']
    elif plotoption =='ylin':
        plotselect=['Y Acceleration']
    elif plotoption == 'zlin':
        plotselect= ['Z Acceleration']
    elif plotoption == "xrot":
        plotselect = ['X Rotation']
    elif plotoption == 'yrot':
        plotselect = ['Y Rotation']
    elif plotoption=='zrot':
        plotselect=['Z Rotation']

    print("Selected plots are ", plotselect)
   
    
    for file in files:
        thisdata = get_csv_data(file, breakpoints_file, options, rezero)
        if thisdata is not None:
            datasets.append(thisdata)
    plot_data(datasets, " & " .join(substrings),plotselect, options)

def plotall():
    directory = sys.argv[1] if len(sys.argv) > 1 else input("Enter directory to look for CSV files: ")
    files = get_files(directory, 'csv')
    breakpoints_file = sys.argv[2] if len(sys.argv) > 2 else None

    cards = ["beaker","M","dt","pentagon","parasail","jetski"]
    # paired groups with matched cards for hhi/hri
    participants = ["group1-|group4","group2|group17","group3|group6","group4|group7","group5|group8","group6|group3","group7|group10","group8|group5","group9|group12","group10|group7","group11|group8","group12|group9","group13|group10","group14|group5","group15|group12","group16|group7","group17|group14"]

    # by group 
    #participants = ["group1-","group2","group3","group4","group5","group6","group7","group8","group9","group10","group11","group12","group13","group14","group15","group16","group17"]
    
    options = "segment"
    rezero=True
    plotselect=['X Acceleration', 'Y Acceleration', 'Z Acceleration']
    

    for card in cards:
        for participant in participants:
            substrings = [card,participant]
            print("filtering based on",substrings)
            
            # Filter the files based on the parsed filter groups
            filter_groups = [group.split('|') for group in substrings]            
            plotfiles = [f for f in files if all(any(sub in f for sub in group) for group in filter_groups)]

            #plotfiles = [f for f in files if all(group in f for group in filter_groups)]
            print("files are ", plotfiles)

            datasets = []
            
            for file in plotfiles:
                thisdata = get_csv_data(file, breakpoints_file, options, rezero)
                if thisdata is not None:
                    datasets.append(thisdata)
            try:
                plot_data(datasets, " & " .join([card,participant]),plotselect, options, save=True)
            except:
                print("no data in set")

def shadeplot_bycard():
    directory = sys.argv[1] if len(sys.argv) > 1 else input("Enter directory to look for CSV files: ")
    files = get_files(directory, 'csv')
    breakpoints_file = sys.argv[2] if len(sys.argv) > 2 else None

    cards = ["beaker","M","dt","pentagon","parasail","jetski"]
    options = "segment"
    rezero=True
    plotselect=['X Acceleration', 'Y Acceleration', 'Z Acceleration']
    interaction_types = ['hh','robotleader','humanleader']
    for card in cards:
        for interaction in interaction_types:
            
            substrings = [card,interaction]
            print("filtering based on",substrings)
    
            # Filter the files based on the parsed filter groups
            filter_groups = [group.split('|') for group in substrings]            
            plotfiles = [f for f in files if all(any(sub in f for sub in group) for group in filter_groups)]
    
            #plotfiles = [f for f in files if all(group in f for group in filter_groups)]
            print("files are ", plotfiles)
    
            datasets = []
    
            for file in plotfiles:
                thisdata = get_csv_data(file, breakpoints_file, options, rezero)
                if thisdata is not None:
                    datasets.append(thisdata)
            plot_data(datasets, " & " .join([card,interaction]),plotselect, options, save=True)
            plot_mean_shaded(datasets, " & " .join([card,interaction]),plotselect, options, save=True)

 

    
if __name__ == "__main__":
    #plotall()
    #main()
    shadeplot_bycard()
