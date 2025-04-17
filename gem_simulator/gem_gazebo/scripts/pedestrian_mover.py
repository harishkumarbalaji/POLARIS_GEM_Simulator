#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
import math

def main():
    rospy.init_node('pedestrian_mover')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    model_name = 'person_walking'
    speed = rospy.get_param('~speed', 1)
    distance_limit = rospy.get_param('~distance_limit', 5.0)
    rate = rospy.Rate(10)  # 10 Hz

    x_start = rospy.get_param('~x_start', -10.0)
    y_start = rospy.get_param('~y_start', -16.0)
    z_pos = rospy.get_param('~z_pos', 0.0)

    yaw_forward = 0.0
    yaw_backward = math.pi

    # Start by moving in negative Y direction
    direction = -1
    y_pos = y_start
    yaw = yaw_forward if direction == -1 else yaw_backward

    while not rospy.is_shutdown():
        y_pos += direction * speed * 0.1  # move in current direction

        # Check if distance limit reached, reverse direction
        if abs(y_pos - y_start) >= distance_limit:
            direction *= -1
            yaw = yaw_forward if direction == -1 else yaw_backward

        pose = Pose()
        pose.position.x = x_start
        pose.position.y = y_pos
        pose.position.z = z_pos
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        state = ModelState()
        state.model_name = model_name
        state.pose = pose
        state.twist = Twist()
        state.reference_frame = "world"

        try:
            set_state(state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set model state: {e}")

        rate.sleep()

if __name__ == "__main__":
    main()

