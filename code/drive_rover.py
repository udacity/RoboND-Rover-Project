# Do the necessary imports
import argparse
import shutil
import base64
from datetime import datetime
import os
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import json
import pickle
import matplotlib.image as mpimg
import time

# Import functions for perception and decision making
from perception import perception_step
from decision import decision_step
from output_images import create_output_images
# Initialize socketio server and Flask application 
# (learn more at: https://python-socketio.readthedocs.io/en/latest/)
sio = socketio.Server()
app = Flask(__name__)

# Read in ground truth map and create 3-channel green version for overplotting
# NOTE: images are read in by default with the origin (0, 0) in the upper left
# and y-axis increasing downward.
ground_truth = mpimg.imread('../calibration_images/map_bw.png')
# This next line creates arrays of zeros in the red and blue channels
# and puts the map into the green channel.  This is why the underlying 
# map output looks green in the display image
ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)

# Define RoverState() class to retain rover state parameters
class RoverState():
    def __init__(self):
        self.start_time = None
        self.total_time = None
        self.img = None # Current camera image
        self.pos = None # Current position (x, y)
        self.yaw = None # Current yaw angle
        self.pitch = None # Current pitch angle
        self.roll = None # Current roll angle
        self.vel = None # Current velocity
        self.steer = 0 # Current steering angle
        self.throttle = 0 # Current throttle value
        self.brake = 0 # Current brake value
        self.nav_angles = None # Angles of navigable terrain pixels
        self.nav_dists = None # Distances of navigable terrain pixels
        self.ground_truth = ground_truth_3d # Ground truth worldmap
        self.mode = 'forward' # Current mode (can be forward or stop)
        self.throttle_set = 0.2 # Throttle setting when accelerating
        self.brake_set = 10 # Brake setting when braking
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!
        self.stop_forward = 50 # Threshold to initiate stopping
        self.go_forward = 500 # Threshold to go forward again
        self.max_vel = 2 # Maximum velocity (meters/second)
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float) 
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float) 
        self.samples_pos = None
        self.samples_found = None
# Initialize our rover 
Rover = RoverState()


# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        global Rover
        
        # Initialize start time and sample positions
        if Rover.start_time == None:
            Rover.start_time = time.time()
            Rover.total_time = 0
            samples_xpos = np.int_([np.float(pos.strip()) for pos in data["samples_x"].split(',')])
            samples_ypos = np.int_([np.float(pos.strip()) for pos in data["samples_y"].split(',')])
            Rover.samples_pos = (samples_xpos, samples_ypos)
            Rover.samples_found = np.zeros((len(Rover.samples_pos[0]))).astype(np.int)
        # Or just update elapsed time
        else:
            tot_time = time.time() - Rover.start_time
            if np.isfinite(tot_time):
                Rover.total_time = tot_time
        # Print out the fields in the telemetry data dictionary
        print(data.keys())
        # The current speed of the rover in m/s
        Rover.vel = np.float(data["speed"])
        # The current position of the rover
        Rover.pos = np.fromstring(data["position"], dtype=float, sep=',')
        # The current yaw angle of the rover
        Rover.yaw = np.float(data["yaw"])
        # The current yaw angle of the rover
        Rover.pitch = np.float(data["pitch"])
        # The current yaw angle of the rover
        Rover.roll = np.float(data["roll"])
        # The current throttle setting
        Rover.throttle = np.float(data["throttle"])
        # The current steering angle
        Rover.steer = np.float(data["steering_angle"])

        print('speed =',Rover.vel, 'position =', Rover.pos, 'throttle =', 
            Rover.throttle, 'steer_angle =', Rover.steer)
  
        # Get the current image from the center camera of the rover
        imgString = data["image"]
        image = Image.open(BytesIO(base64.b64decode(imgString)))
        Rover.img = np.asarray(image)

        if np.isfinite(Rover.vel):
            
            # Execute the perception and decision steps to update the Rover's state
            Rover = perception_step(Rover)
            Rover = decision_step(Rover)

            # Create output images to send to server
            out_image_string1, out_image_string2 = create_output_images(Rover)

            # The action step!  Send commands to the rover!
            
            commands = (Rover.throttle, Rover.brake, Rover.steer)
            #print(commands)
            send_control(commands, out_image_string1, out_image_string2)

        else:

            # Send zeros for throttle, brake and steer and empty images
            # if we got a bad velocity value in telemetry
            send_control((0, 0, 0), '', '')
        
        # Save image frame if folder was specified
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = os.path.join(args.image_folder, timestamp)
            image.save('{}.jpg'.format(image_filename))
    else:
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control((0, 0, 0), '', '')
    sample_data = {}
    sio.emit(
        "get_samples",
        sample_data,
        skip_sid=True)

def send_control(commands, image_string1, image_string3):
    #if image_string != '':
    data={
        'throttle': commands[0].__str__(),
        'brake': commands[1].__str__(),
        'steering_angle': commands[2].__str__(),
        'inset_image': image_string1,
        'inset_image3': image_string3,
        }

    sio.emit(
        "steer",
        data,
        skip_sid=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()
    
    os.system('rm -rf IMG_stream/*')
    if args.image_folder != '':
        print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        print("Recording this run ...")
    else:
        print("NOT recording this run ...")
    
    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
