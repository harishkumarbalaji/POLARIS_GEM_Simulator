import math

# ROS Headers
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import tf.transformations

from septentrio_gnss_driver.msg import INSNavGeod

class INSPublisher(object):

    def __init__(self):
        self.latest_gps = None
        self.latest_imu = None

        # Subscribers
        self.gps_sub = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)

        # Publisher
        self.ins_pub = rospy.Publisher("/insnavgeod", INSNavGeod, queue_size=10)


    def gps_callback(self, msg):
        """Sets latest GPS data publishes combined INSNavGeod message"""
        self.latest_gps = msg
        self.publish_combined_data()

    def imu_callback(self, msg):
        """Sets latest IMU data"""
        self.latest_imu = msg

    def publish_combined_data(self):
        """
        Combines the latest GPS and IMU data into an INSNavGeod message
        and publishes it.
        """
        current_gps = self.latest_gps
        current_imu = self.latest_imu

        if not current_gps or not current_imu:
            return

        ins_msg = INSNavGeod()
        ins_msg.header.stamp = current_gps.header.stamp

        ins_msg.latitude = current_gps.latitude
        ins_msg.longitude = current_gps.longitude
        ins_msg.height = current_gps.altitude

        # convert orientation to roll, pitch, yaw
        q = current_imu.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # convert to degrees
        ins_msg.roll, ins_msg.pitch, ins_msg.heading = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

        self.ins_pub.publish(ins_msg)

# Main execution
if __name__ == '__main__':
    try:
        rospy.init_node('ins_publisher_node', anonymous=True)
        ins_publisher_object = INSPublisher()
        rospy.spin()

    except Exception as e:
        rospy.logerr(f"An error occurred in the INS Publisher node: {e}")