#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 16 14:56:03 2019

@author: clochette
"""

import roslib
import numpy as np
import cv2
import time
import os
import argparse
import rospy
import socket
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

"""
-Init node as a publisher over selected topic
-Read images in sequence from the selected path
-Translate images to messages
-Send over selected topic
"""

parser = argparse.ArgumentParser(description='Take images from a saved sequence and send them over a topic.')
parser.add_argument('--topic', action='store', default='/camera/image_raw')
parser.add_argument('--src', action='store', default='./data/')
parser.add_argument('--fps', action='store', default=4, type=int)
parser.add_argument('--prefix', action='store', default='image_')
parser.add_argument('--start', action='store', default=0, type=int)
parser.add_argument('--ext', action='store', default='jpg')

args = parser.parse_args()

topic = args.topic
freq = args.fps
src_dir = args.src

bridge = CvBridge()
publisher = rospy.Publisher(topic, Image, queue_size=10)
rospy.init_node('slam_reader', anonymous=True)

image_id = args.start
prefix_full = args.src + args.prefix
tmp = raw_input("Waiting")
ext = args.ext
expected_delay = 1.0 / args.fps
prev_time = time.time()
while True:
    name = prefix_full + str(image_id) + '.' + ext
    #Read image
    image = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
    #If no image found, break
    if image is None:
        break
    else:
    #else
        #Translate into ROS message
        #publish image
        publisher.publish(bridge.cv2_to_imgmsg(image, "mono8"))
        #increase counter
        image_id = image_id + 1

    cur_time = time.time()
    delta_time = cur_time - prev_time
    prev_time = cur_time
    if delta_time < expected_delay:
        time.sleep(expected_delay - delta_time)