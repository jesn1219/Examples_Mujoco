import sys
from os import path

if __package__ is None:
    #print("__package__ is None") 
    print(path.dirname( path.dirname( path.abspath('__file__') ) ))
    sys.path.append(path.dirname( path.dirname( path.abspath("__file__") ) ))
from Jutils import display_video

import mujoco_py
import copy
import os
import itertools
from IPython.display import clear_output, display, Image, HTML
import numpy as np
# Graphics-related
import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import PIL.Image

def set_cam(env, config) :
    env.viewer.cam.distance = config['distance']
    env.viewer.cam.azimuth = config['azimuth'] # angle for horizontal
    env.viewer.cam.elevation = config['elevation'] # angle for vertical

import custom_gym as cg

env_name = 'FetchAnt-v1'
env = cg.make(env_name) # Initalize with 'FetchAnt-v1' - embedded in Gym
env.reset()

cam_config_1 = {'distance' : 2.5, 'azimuth' : 180.0, 'elevation' : -14.0}
cam_config_2 = {'distance' : 2.5, 'azimuth' : 64.0, 'elevation' : -14.0}
#env.viewer.move_camera()

video_1 = []
while(1):
    obs, reward, done, info = env.step(env.action_space.sample()) # take a random action
    pixels = env.render()
    #set_cam(env, cam_config_1)
    #env.viewer.move_camera(1,-0.01*i, 0) # move_camera(action, dx / height, dy / height)
    #video_1.append(env.render(mode='rgb_array').copy())

#print("Createing Video..")
#display_video(video_1, gif=True)