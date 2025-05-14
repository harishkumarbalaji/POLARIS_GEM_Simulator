import argparse

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

from util import euler_to_quaternion

GEM_E2 = "gem_e2"
GEM_E4 = "gem_e4"

def getModelState(vehicle_name):
    print(f"Getting state for vehicle: {vehicle_name}")
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        modelState = serviceResponse(model_name=vehicle_name)
    except rospy.ServiceException as exc:
        rospy.loginfo("Service did not process request: "+str(exc))
    return modelState

def setModelState(model_state):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(model_state)
    except rospy.ServiceException as e:
        rospy.loginfo("Service did not process request: "+str(e))

def set_position(x = 0,y = 0, yaw=0, vehicle_name = GEM_E4):
    curr_state = getModelState(vehicle_name)
    new_state = ModelState()

    new_state.model_name = vehicle_name
    new_state.twist.linear.x = 0
    new_state.twist.linear.y = 0
    new_state.twist.linear.z = 0
    new_state.pose.position.x = x
    new_state.pose.position.y = y
    new_state.pose.position.z = -1
   
    q = euler_to_quaternion([0,0,yaw])
    new_state.pose.orientation.x = q[0]
    new_state.pose.orientation.y = q[1]
    new_state.pose.orientation.z = q[2]
    new_state.pose.orientation.w = q[3]
    new_state.twist.angular.x = 0
    new_state.twist.angular.y = 0
    new_state.twist.angular.z = 0
    setModelState(new_state)

if __name__ == "__main__":
    # Initialize the ROS node first, before accessing parameters
    rospy.init_node("set_pos")
    
    parser = argparse.ArgumentParser(description = 'Set the x, y position of the vehicle')
    x_default = 17.81
    y_default = -10
    yaw_default = 3.14

    parser.add_argument('--x', type = float, help = 'x position of the vehicle.', default = x_default)
    parser.add_argument('--y', type = float, help = 'y position of the vehicle.', default = y_default)
    parser.add_argument('--yaw', type = float, help = 'yaw of the vehicle.', default = yaw_default)
    parser.add_argument('--vehicle', type = str, help = 'vehicle name (gem_e4 or gem_e2)', default = GEM_E4)

    argv = parser.parse_args()

    x = argv.x
    y = argv.y
    yaw = argv.yaw
    vehicle_name = argv.vehicle

    set_position(x = x, y = y, yaw = yaw, vehicle_name = vehicle_name)
