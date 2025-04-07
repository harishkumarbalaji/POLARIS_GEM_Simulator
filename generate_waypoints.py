#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
import threading
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.srv import GetModelState
from datetime import datetime
import numpy as np

class AckermannTeleop:
    def __init__(self):
        rospy.init_node('ackermann_keyboard_teleop')

        self.model_name = "gem_e4"  # Replace with your model name in Gazebo
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.cmd_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)

        self.speed = 0.0
        self.steering_angle = 0.0
        self.drive_msg = AckermannDrive()

        self.waypoints = []
        self.last_record_time = rospy.Time.now()

        self.running = True
        self.rate = rospy.Rate(10)

        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def get_vehicle_position(self):
        try:
            res = self.get_model_state(self.model_name, '')
            pos = res.pose.position
            def quaternion_to_euler(x, y, z, w):
                t0 = +2.0 * (w * x + y * z)
                t1 = +1.0 - 2.0 * (x * x + y * y)
                roll = np.arctan2(t0, t1)
                t2 = +2.0 * (w * y - z * x)
                t2 = +1.0 if t2 > +1.0 else t2
                t2 = -1.0 if t2 < -1.0 else t2
                pitch = np.arcsin(t2)
                t3 = +2.0 * (w * z + x * y)
                t4 = +1.0 - 2.0 * (y * y + z * z)
                yaw = np.arctan2(t3, t4)
                return [roll, pitch, yaw]
            _,_, yaw = quaternion_to_euler(res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w)

            return round(pos.x, 2), round(pos.y, 2)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")
            return None

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def keyboard_loop(self):
        print("Control: W/S - forward/back | A/D - left/right | Q - quit")
        while self.running and not rospy.is_shutdown():
            key = self.get_key()
            if key == 'w':
                self.speed += 0.2
            elif key == 's':
                self.speed -= 0.2
            elif key == 'a':
                self.steering_angle += 0.05
            elif key == 'd':
                self.steering_angle -= 0.05
            elif key == 'q':
                self.running = False
                break

            self.publish_drive()

    def publish_drive(self):
        self.drive_msg.speed = self.speed
        self.drive_msg.steering_angle = self.steering_angle
        self.cmd_pub.publish(self.drive_msg)

    def record_waypoint(self):
        now = rospy.Time.now()
        if (now - self.last_record_time).to_sec() > 0.5:
            pos = self.get_vehicle_position()
            if pos:
                self.waypoints.append(pos)
                self.last_record_time = now
                rospy.loginfo(f"Waypoint recorded: {pos}")

    def run(self):
        while not rospy.is_shutdown() and self.running:
            self.record_waypoint()
            self.rate.sleep()

        rospy.loginfo("Stopped. Final waypoints:")
        for wp in self.waypoints:
            print(wp[0]+1.5, ",",wp[1]+21)

if __name__ == '__main__':
    try:
        teleop = AckermannTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
