#!/usr/bin/env python3

from __future__ import print_function

# Python Headers
import os
import cv2 
import csv
import math
import numpy as np
from numpy import linalg as la

# ROS Headers
import tf
import rospy
import rospkg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM Sensor Headers
from std_msgs.msg import Float64
from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu, NavSatFix
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva, NovatelCorrectedImuData
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod

image_file  = 'gnss_map.png'
curr_path = os.path.abspath(__file__) 
image_path = curr_path.split('gem_simulator')[0] + 'images/' + image_file

class GNSSImage(object):

    global image_path
    
    def __init__(self):

        self.rate = rospy.Rate(15)

        # Read image in BGR format
        self.map_image = cv2.imread(image_path)

        # Create the cv_bridge object
        self.bridge  = CvBridge()
        self.map_image_pub = rospy.Publisher("/motion_image", Image, queue_size=1) 

        # Get vehicle_name parameter
        self.vehicle_name = rospy.get_param('~vehicle_name', 'gem_e4')
        rospy.loginfo(f"GNSSImage: Using vehicle_name: {self.vehicle_name}")
        
        # Subscribe to appropriate topic based on vehicle_name
        self.lat = 0
        self.lon = 0
        self.heading = 0
        
        if self.vehicle_name == 'gem_e2':
            self.ins_sub = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
            rospy.loginfo("Subscribed to /novatel/inspva topic with Inspva message for gem_e2")
        else:
            self.ins_sub = rospy.Subscriber("/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)
            rospy.loginfo("Subscribed to /septentrio_gnss/insnavgeod topic with INSNavGeod message for gem_e4")

        self.lat_start_bt = 40.092722  # 40.09269  
        self.lon_start_l  = -88.236365 # -88.23628
        self.lat_scale    = 0.00062    # 0.0007    
        self.lon_scale    = 0.00136    # 0.00131   

        self.arrow        = 40 
        self.img_width    = 2107
        self.img_height   = 1313

    def inspva_callback(self, msg):
        """Callback for Inspva messages from gem_e2"""
        self.heading = round(msg.azimuth, 1)
        self.lat = msg.latitude
        self.lon = msg.longitude

    def ins_callback(self, msg):
        """Callback for INSNavGeod messages from gem_e4"""
        self.heading = round(msg.heading, 1) 
        self.lat = msg.latitude
        self.lon = msg.longitude

    def image_heading(self, lon_x, lat_y, heading):
        
        if(heading >=0 and heading < 90):
            angle  = np.radians(90-heading)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle))

        elif(heading >= 90 and heading < 180):
            angle  = np.radians(heading-90)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))  

        elif(heading >= 180 and heading < 270):
            angle = np.radians(270-heading)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))

        else:
            angle = np.radians(heading-270)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle)) 

        return lon_xd, lat_yd         


    def start_gi(self):
        
        while not rospy.is_shutdown():

            lon_x = int(self.img_width*(self.lon-self.lon_start_l)/self.lon_scale)
            lat_y = int(self.img_height-self.img_height*(self.lat-self.lat_start_bt)/self.lat_scale)
            lon_xd, lat_yd = self.image_heading(lon_x, lat_y, self.heading)
            with open(os.path.join(curr_path.split('gem_gnss_image.py')[0], 'image_info.txt'), 'a') as f:
                f.write(f"lon_x: {lon_x}, lat_y: {lat_y}, lon_xd: {lon_xd}, lat_yd: {lat_yd}\n")

            pub_image = np.copy(self.map_image)

            if(lon_x >= 0 and lon_x <= self.img_width and lon_xd >= 0 and lon_xd <= self.img_width and 
                lat_y >= 0 and lat_y <= self.img_height and lat_yd >= 0 and lat_yd <= self.img_height):
                cv2.arrowedLine(pub_image, (lon_x, lat_y), (lon_xd, lat_yd), (0, 0, 255), 2)
                cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)

            try:
                # Convert OpenCV image to ROS image and publish
                self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

            self.rate.sleep()


def main():

    rospy.init_node('gem_gnss_image_node', anonymous=True)

    gi = GNSSImage()

    try:
        gi.start_gi()
    except KeyboardInterrupt:
        print ("Shutting down gnss image node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

