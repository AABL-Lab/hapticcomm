#hapticstudylibrary.py
from boto3 import Session
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import os
import sys
import subprocess
from tempfile import gettempdir
sys.path.append('/home/tufts_user/catkin_ws/src/armpy/src')
import csv
import armpy


def robotintroduction(robotname="Beep"):


    # Create a client using the credentials and region defined in the [default]
    # section of the AWS credentials file (~/.aws/credentials).
    session = Session(profile_name="default")
    polly = session.client("polly")

    introtext = "Hello my name is" + robotname 
    try:
        # Request speech synthesis
        response = polly.synthesize_speech(Text=introtext, OutputFormat="mp3",
                                            VoiceId="Justin") # young-sounding voice
    except (BotoCoreError, ClientError) as error:
        # The service returned an error, exit gracefully
        print(error)
        sys.exit(-1)

    # Access the audio stream from the response
    if "AudioStream" in response:
        # Note: Closing the stream is important because the service throttles on the
        # number of parallel connections. Here we are using contextlib.closing to
        # ensure the close method of the stream object will be called automatically
        # at the end of the with statement's scope.
        with closing(response["AudioStream"]) as stream:
                output = os.path.join(gettempdir(), "speech.mp3")

        try:
            # Open a file for writing the output as a binary stream
                with open(output, "wb") as file:
                    file.write(stream.read())
                print("wrote audio output to ", output)
        except IOError as error:
            # Could not write to file, exit gracefully
            print(error)
            sys.exit(-1)
        return output
    else:
        # The response didn't contain audio data, exit gracefully
        print("Could not stream audio")
        sys.exit(-1)

def create_trajectory_from_waypoints(filename):
    # filename should be a CSV file, formatted like waypointgathering.py
    print("Loading waypoints from", filename)

    filelist = csv.reader(filename)
    for row in filelist:
        print(row)
        
    print("Moving to start position")
    armpy.move_to_joint_pose(startposition)

    print("Select the points to use in the trajectory")
    while point in filelist: 
        point = str(raw_input())
        waypointlist = waypointlist + point
    print("Generating trajectories from waypoints")
    armpy.plan_waypoints(waypointlist)
        
        
if __name__=="__main__":
    print("This is a library file")
    create_trajectory_from_waypoints("waypoints.csv")
