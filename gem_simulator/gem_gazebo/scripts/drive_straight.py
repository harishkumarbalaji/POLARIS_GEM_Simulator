#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDrive


def main():
    rospy.init_node("drive_straight")

    topic = rospy.get_param("~topic", "/ackermann_cmd")
    speed = float(rospy.get_param("~speed", 2.0))
    rate_hz = float(rospy.get_param("~rate", 10.0))
    duration = float(rospy.get_param("~duration", 0.0))
    acceleration = float(rospy.get_param("~acceleration", 0.0))
    steering_angle_velocity = float(rospy.get_param("~steering_angle_velocity", 0.0))
    jerk = float(rospy.get_param("~jerk", 0.0))

    pub = rospy.Publisher(topic, AckermannDrive, queue_size=1)
    rate = rospy.Rate(rate_hz)

    msg = AckermannDrive()
    msg.steering_angle = 0.0
    msg.speed = speed
    msg.acceleration = acceleration
    msg.steering_angle_velocity = steering_angle_velocity
    msg.jerk = jerk

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        if duration > 0.0:
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break

        pub.publish(msg)
        rate.sleep()

    stop_msg = AckermannDrive()
    pub.publish(stop_msg)


if __name__ == "__main__":
    main()
