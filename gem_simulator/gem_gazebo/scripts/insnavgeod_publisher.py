#!/usr/bin/env python3
import math

# ROS Headers
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3Stamped
from gazebo_msgs.msg import ModelStates
import tf.transformations

from septentrio_gnss_driver.msg import INSNavGeod

GEM_E2 = "gem_e2"
GEM_E4 = "gem_e4"
GEM_E2_GNSS_TOPIC = "/lidar1/velodyne_points"
GEM_E4_GNSS_TOPIC = "/ouster/points"

class INSNavGeodPublisher(object):

    def __init__(self):
        self.latest_gps = None
        self.latest_imu = None
        self.latest_gps_velocity = None
        self.vehicle = GEM_E4 # default is to assume gem_e4

        # Subscribers
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        self.gps_sub = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.gps_vel_sub = rospy.Subscriber("/gps/fix_velocity", Vector3Stamped, self.gps_vel_callback)

        # Publisher (default is gem_e4 topic name)
        self.insnavgeod_pub = rospy.Publisher(GEM_E4_GNSS_TOPIC, INSNavGeod, queue_size=10)


    def model_states_callback(self, msg: ModelStates):
        """Finds the current vehicle model in Gazebo and changes published topic name accordingly"""
        if self.vehicle not in msg.name:
            if GEM_E2 in msg.name:
                self.vehicle = GEM_E2
                self.insnavgeod_pub = rospy.Publisher(GEM_E2_GNSS_TOPIC, INSNavGeod, queue_size=10)
            else:
                self.vehicle = GEM_E4
                self.insnavgeod_pub = rospy.Publisher(GEM_E4_GNSS_TOPIC, INSNavGeod, queue_size=10)

    def gps_callback(self, msg):
        """Sets latest GPS data publishes combined INSNavGeod message"""
        self.latest_gps = msg
        self.publish_combined_data()

    def imu_callback(self, msg):
        """Sets latest IMU data"""
        self.latest_imu = msg

    def gps_vel_callback(self, msg):
        """
        Sets latest GPS velocity data.
        """
        self.latest_gps_velocity = msg

    def publish_combined_data(self):
        """
        Combines the latest GPS and IMU data into an INSNavGeod message
        and publishes it.
        """
        current_gps = self.latest_gps
        current_imu = self.latest_imu
        current_gps_velocity = self.latest_gps_velocity

        if not current_gps or not current_imu or not current_gps_velocity:
            return

        msg = INSNavGeod()
        msg.header.stamp = current_gps.header.stamp
        msg.header.frame_id = current_gps.header.frame_id
        msg.error = current_gps.status.status

        msg.latitude = current_gps.latitude
        msg.longitude = current_gps.longitude
        msg.height = current_gps.altitude

        # convert orientation to roll, pitch, yaw
        q = current_imu.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # convert to degrees
        msg.roll, msg.pitch, msg.heading = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

        msg.ve, msg.vn, msg.vu = current_gps_velocity.vector.x, current_gps_velocity.vector.y, current_gps_velocity.vector.z

        self.insnavgeod_pub.publish(msg)

# Main execution
if __name__ == '__main__':
    try:
        rospy.init_node('insnavgeod_publisher_node', anonymous=True)
        ins_publisher_object = INSNavGeodPublisher()
        rospy.spin()

    except Exception as e:
        rospy.logerr(f"An error occurred in the INSNavGeod Publisher node: {e}")
