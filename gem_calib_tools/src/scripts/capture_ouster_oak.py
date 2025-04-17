#!/usr/bin/env python3
"""
Capture LiDAR and Camera Data from Simulation (Ouster configuration adapted)

This script subscribes to LiDAR and camera topics from your simulation,
then allows you to save a snapshot of the sensor data by pressing a key.
It saves:
  - LiDAR point cloud (npz format)
  - Camera image (PNG format)
  - (Optional) Depth image (if available) as a 16-bit tif

Press 'Escape' (or Ctrl+C) to quit.
"""

import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

# Global variables for messages
lidar_points = None
camera_image = None
# depth_image is optional – if not published in your simulation, this remains None.
depth_image = None
bridge = CvBridge()

def lidar_callback(lidar: PointCloud2):
    global lidar_points
    lidar_points = lidar

def camera_callback(img: Image):
    global camera_image
    camera_image = img

# Optional: If you have a depth topic uncomment the following
# def depth_callback(img: Image):
#     global depth_image
#     depth_image = img

def pc2_to_numpy(pc2_msg, want_rgb=False):
    gen = pc2.read_points(pc2_msg, skip_nans=True)
    if want_rgb:
        xyzpack = np.array(list(gen), dtype=np.float32)
        if xyzpack.shape[1] != 4:
            raise ValueError("PointCloud2 does not have points")
        xyzrgb = np.empty((xyzpack.shape[0], 6))
        xyzrgb[:, :3] = xyzpack[:, :3]
        for i, x in enumerate(xyzpack):
            rgb = x[3]
            s = struct.pack('>f', rgb)
            i_val = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i_val).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            xyzrgb[i, 3:] = (r, g, b)
        return xyzrgb
    else:
        return np.array(list(gen), dtype=np.float32)[:, :3]

def save_data(lidar_file, image_file):
    global lidar_points, camera_image
    if lidar_points and camera_image:
        # Save LiDAR data
        lidar_np = pc2_to_numpy(lidar_points)
        np.savez(lidar_file, lidar_np)
        # Save Image data
        cv_image = bridge.imgmsg_to_cv2(camera_image, "bgr8")
        cv2.imwrite(image_file, cv_image)
        print("Saved:", lidar_file, image_file)
    else:
        print("Missing sensor data; not saving.")

def main(folder='/home/manan/host/gem_sim/src/POLARIS_GEM_e2_Simulator/gem_calib_tools/src/data', start_index=1):
    rospy.init_node('capture_ouster_oak', anonymous=True)
    # Update subscriptions to match your simulation topics:
    rospy.Subscriber("/front_single_camera/image_raw", Image, camera_callback)
    rospy.Subscriber("/velodyne_points", PointCloud2, lidar_callback)
    # If depth is available, you can subscribe as follows:
    # rospy.Subscriber("/depth_topic", Image, depth_callback)
    
    if not os.path.exists(folder):
        os.makedirs(folder)
    
    index = start_index
    print("Press 's' to save a snapshot, 'q' to quit.")
    while not rospy.is_shutdown():
        if camera_image is not None:
            cv2.imshow("Camera Image", bridge.imgmsg_to_cv2(camera_image, "bgr8"))
            key = cv2.waitKey(30)
            if key == ord('q'):
                break
            elif key == ord('s'):
                if lidar_points is None or camera_image is None:
                    print("Missing some sensor data.")
                else:
                    lidar_file = os.path.join(folder, f"lidar_{index}.npz")
                    image_file = os.path.join(folder, f"image_{index}.png")
                    save_data(lidar_file, image_file)
                    index += 1
    cv2.destroyAllWindows()

if __name__ == '__main__':
    import sys
    folder = '/home/manan/host/gem_sim/src/POLARIS_GEM_e2_Simulator/gem_calib_tools/src/data'
    start_index = 1
    if len(sys.argv) >= 2:
        folder = sys.argv[1]
    if len(sys.argv) >= 3:
        start_index = int(sys.argv[2])
    main(folder, start_index)
