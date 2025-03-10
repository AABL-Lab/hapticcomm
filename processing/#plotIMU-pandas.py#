import matplotlib.pyplot as plt
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
import numpy as np
import glob
import os
import sys
from datetime import datetime, timedelta, date
import itertools

import re

def get_csv_data(filename, breakpointfilename=None):
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
    
    
    df = pd.read_csv(filename, skiprows=1, names=["timestamp", "Xaccel", "Yaccel", "Zaccel", "Xrotation", "Yrotation", "Zrotation"], engine='python')
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
    print(f"Group: {group}, Subgroup: {subgroup}, Card: {card}")




    ## If breakpoints were provided in a file, we will pull them and some other timing data from there
    if breakpointfilename:
        breakpoint_df = pd.read_csv(breakpointfilename)
        print(f"Attempting to read breakpoint file: {breakpointfilename}")
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
        print("Breakpoints after firstmatch are ", breakpoints)

        breakpoints = [float(bp) for bp in breakpoints]
        duration = breakpoints[-1] if breakpoints else df['timestamp'].iloc[-1]

        index_list = matched_bp.index.tolist()
        print("Index of matched_bp", index_list)
        trial_starttime = str(matched_bp.loc[index_list[0],'24hStart'])
        trial_starttime  = trial_starttime.replace(" ", "")  # Remove all spaces
        print("Trial Start Time ", trial_starttime)
        trial_starttime = datetime.strptime(trial_starttime, "%H:%M:%S").time()


        print("Trial duration is ", duration)
        print("Breakpoints are ", breakpoints)


        
    else:
        duration, breakpoints, trial_starttime = get_manual_segment_data()

    waittime = datetime.combine(date.min,trial_starttime)-  datetime.combine(date.min, clocktimeIMUstart)
    print("difference between trial start time", trial_starttime, " and IMU clock start time ", clocktimeIMUstart," is ", waittime)

    df, scaledbreakpoints = rescale_by_segment(df, breakpoints, waittime, filename)
    #df, scaledbreakpoints = rescale_alldata_1to100(df, breakpoints, waittime, filename)


        
    return {'name': filename, 'data': df, 'breakpoints': scaledbreakpoints, 'duration': duration, 'clocktimeIMUstart': clocktimeIMUstart}



def rescale_by_segment(df, breakpoints, waittime, filename):
    # for each segment, we want to rescale first 1 to 100 (to deal with the time being in seconds and the data not)
    # and then by breakpoint so that we can try to even the breakpoints out.
    df, breakpoints = rescale_alldata_1to100(df, breakpoints, waittime, filename)



    scaled_segments = []
    
    # Process the first segment (before the first breakpoint, negative time)
    first_segment = df[df['time'] < breakpoints[0]].copy()
    first_segment.loc[:, 'segmentscaledtime'] =  np.interp(first_segment['time'], (df['time'].min(), breakpoints[0]), (-1,0))
    scaled_segments.append(first_segment)
    print(first_segment)
    
    # Process all intermediate segments
    for i in range(len(breakpoints) - 1):
        lower, upper = breakpoints[i], breakpoints[i + 1]
        segment = df[(df['time'] >= lower) & (df['time'] < upper)].copy()  # Explicit copy
        if not segment.empty:
            segment.loc[:, 'segmentscaledtime'] = np.interp(segment['time'], (lower, upper), (i , (i + 1) ))
            scaled_segments.append(segment)
        
    # Process the last segment (after the last breakpoint) if any
    last_segment = df[df['time'] >= breakpoints[-1]].copy()
    last_segment.loc[:, 'segmentscaledtime'] = np.interp(last_segment['time'], (breakpoints[-1], df['time'].max()), (len(breakpoints), (len(breakpoints) + 1) ))
    scaled_segments.append(last_segment)
    
    # Combine all segments into a new DataFrame
    df_scaled = pd.concat(scaled_segments).sort_values('time')
    
    # Reset index
    df_scaled.reset_index(drop=True, inplace=True)
    
    return df_scaled, [0,1,2,3,4,5,6]
        


    
        
def rescale_alldata_1to100(df, breakpoints, waittime, filename):
    # We want all the IMU files to display on the same scale, so we rescale the timestamps over 100 units
    time_start = df['timestamp'].iloc[0]
    time_end = df['timestamp'].iloc[-1]

    # leaving the timestamps aside, we make a new variable called time that ranges from 0-100 and rescale the data within that
    df['time'] = (df['timestamp'] - time_start) / (time_end - time_start) * 100

    # the breakpoints do not include the wait time, so we need to add that to account for the dead time at the start of the trial
    if 0 <= waittime.total_seconds() <= 15:
        
        bp_start = breakpoints[0]
        bp_end = breakpoints[-1]+waittime.total_seconds()
        # add the wait time to each breakpoint, then scale them out of 100
        print("Breakpoints start at ", bp_start, "and end at ", bp_end)
        scaledbreakpoints = [
            ((bp + waittime.total_seconds()) / bp_end) * 100
            for bp in breakpoints
        ]
        print("scaled breakpoints are", scaledbreakpoints)
    else:
        print("Wait time for file ", filename , "may be miscalculated as it is more than 15 seconds, check data")
        scaledbreakpoints = [
            ((bp) / breakpoints[-1]) * 100
            for bp in breakpoints
        ]
        input()
        
    return df, scaledbreakpoints

    # Drop rows where the 'time' is less than the first scaled breakpoint
    #df = df[df['time'] > scaledbreakpoints[0]]
    



def get_manual_segment_data():
    duration = float(input("Enter file duration or leave blank for none: ") or 0)
    breakpoints = input("Enter breakpoints separated by commas or leave blank: ")
    breakpoints = list(map(float, breakpoints.split(','))) if breakpoints else []
    trial_starttime = input("Enter trial start time as HH:MM:SS")
    trial_starttime = datetime.strptime(trial_starttime, "%H:%M:%S").time()

    ##fixme: this will need to also collect file and trial start time offsets if we want to use it to rescale
    return duration, breakpoints, trial_starttime


def plot_data(filenamedictionary_list, title):
    try:
        filenamedictionary_list[0]['data']['segmentscaledtime']
        # X positions for labels
        divider_positions = [-1, 0, 1, 2, 3, 4, 5, 6]
        dividerlabels = ["recording start", "trial start", "end leg 1", "end leg 2", 
                 "end leg 3", "end leg 4", "end leg 5", "end trial"]
    except KeyError:
        divider_positions = []
        dividerlabels = []
    
    # Create a figure with 3 subplots for each axis
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 6))
    fig.suptitle(title)
    labels = ['X Acceleration', 'Y Acceleration', 'Z Acceleration']
    # Generate a list of distinct colors (you can expand this list if you have more datasets)
    color_cycle = itertools.cycle(['r', 'g', 'b', 'c', 'm', 'y', 'k'])  # You can add more colors to the list


    # Loop through each dataset in the list
    for dataset in filenamedictionary_list:
        # Get a color for the current dataset from the color cycle
        color = next(color_cycle)
        # Loop through each label (X, Y, Z)
        for i, dlabel in enumerate(labels):
            # Plot the dataset
            try:
                axes[i].plot(dataset['data']['segmentscaledtime'], dataset['data'][dlabel[:1] + 'accel'], label=dataset['name'], color=color)
            except KeyError:
                axes[i].plot(dataset['data']['time'], dataset['data'][dlabel[:1] + 'accel'], label=dataset['name'], color=color)
            # Add vertical lines and labels
            for pos, slabel in zip(divider_positions, dividerlabels):
                axes[i].axvline(x=pos, color='gray', linestyle='--', alpha=0.7)  # Add vertical line
        
            # Set the ylabel for the current axis
            axes[i].set_ylabel(dlabel)
    # Set common x-label and ticks
    axes[-1].set_xlabel('Segment')
    
    # **Move the label placement outside the dataset loop**
    for pos, slabel in zip(divider_positions, dividerlabels):
        # Place label just above the x-data plot with diagonal rotation
        axes[0].text(pos, axes[0].get_ylim()[1], slabel, 
                      rotation=45, ha='left', va='bottom', fontsize=10, color='black')

    # Extract min and max values from the list of dictionaries
    try:
        min_time = int(min(d['data']['segmentscaledtime'].min() for d in filenamedictionary_list))
        max_time = int(max(d['data']['segmentscaledtime'].max() for d in filenamedictionary_list))
    except KeyError:
        min_time = int(min(d['data']['time'].min() for d in filenamedictionary_list))
        max_time = int(max(d['data']['time'].max() for d in filenamedictionary_list))
        print("min time is ", min_time, "max time is ", max_time)
    
    # Set the x-axis limits to match the data range
    axes[-1].set_xlim(min_time-.5, max_time)
    
    # Set the x-axis ticks properly
    axes[-1].set_xticks(range(min_time, max_time, int(round((max_time-min_time)/10))))

    # Add legend
    plt.legend(loc='upper left',  fontsize='small', bbox_to_anchor=(1.05, 0))
    
    # Adjust layout to avoid overlap
    plt.tight_layout()
    plt.show()
    # def plot_data(filenamedictionary_list, title):


    
def get_files(directory, extension):
    return glob.glob(os.path.join(directory, f'*.{extension}'))


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else input("Enter directory to look for CSV files: ")
    files = get_files(directory, 'csv')
    substrings = input("Enter filename filter criteria (comma-separated): ").split(',')
    print("substrings are ", substrings)
    files = [f for f in files if all(sub in f for sub in substrings)]

    print("Files found are ", files)
    
    breakpoints_file = sys.argv[2] if len(sys.argv) > 2 else None
    datasets = [get_csv_data(file, breakpoints_file) for file in files]
    plot_data(datasets, " & " .join(substrings))


if __name__ == "__main__":
    main()
