#!/usr/bin/env python3
import math

# ROS Headers
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3Stamped
import tf.transformations

from septentrio_gnss_driver.msg import INSNavGeod

GEM_E2 = "gem_e2"
GEM_E4 = "gem_e4"
GEM_E2_GNSS_TOPIC = "/novatel/inspva"
GEM_E4_GNSS_TOPIC = "/septentrio_gnss/insnavgeod"

class INSNavGeodPublisher(object):

    def __init__(self):
        self.latest_gps = None
        self.latest_imu = None
        self.latest_gps_velocity = None
        
        # Get vehicle_name parameter, default to gem_e4
        self.vehicle = rospy.get_param('~vehicle_name', GEM_E4)
        rospy.loginfo(f"INSNavGeodPublisher: Using vehicle_name: {self.vehicle}")
        
        # Set publisher based on vehicle type - using INSNavGeod message for both
        if self.vehicle == GEM_E2:
            self.topic = GEM_E2_GNSS_TOPIC
            rospy.loginfo(f"Publishing to {GEM_E2_GNSS_TOPIC} for gem_e2")
        else:
            self.topic = GEM_E4_GNSS_TOPIC
            rospy.loginfo(f"Publishing to {GEM_E4_GNSS_TOPIC} for gem_e4")
            
        self.pub = rospy.Publisher(self.topic, INSNavGeod, queue_size=10)

        # Subscribers
        self.gps_sub = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.gps_vel_sub = rospy.Subscriber("/gps/fix_velocity", Vector3Stamped, self.gps_vel_callback)

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
        heading = -(math.degrees(yaw)+90)
        if heading > 180:
            heading -= 360
        elif heading < -180:
            heading += 360
        
        msg.roll, msg.pitch, msg.heading = math.degrees(roll), math.degrees(pitch), heading

        msg.ve, msg.vn, msg.vu = current_gps_velocity.vector.x, current_gps_velocity.vector.y, current_gps_velocity.vector.z

        # Publish to the appropriate topic based on vehicle type
        self.pub.publish(msg)

# Main execution
if __name__ == '__main__':
    try:
        rospy.init_node('insnavgeod_publisher_node', anonymous=True)
        ins_publisher_object = INSNavGeodPublisher()
        rospy.spin()

    except Exception as e:
        rospy.logerr(f"An error occurred in the INSNavGeod Publisher node: {e}")
