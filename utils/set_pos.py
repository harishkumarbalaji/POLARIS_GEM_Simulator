import argparse

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

from util import euler_to_quaternion

GEM_E2 = "gem_e2"
GEM_E4 = "gem_e4"

def get_available_models():
    """Get list of available models in the simulation"""
    try:
        model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
        return model_states.name
    except rospy.ROSException as e:
        rospy.loginfo("Error getting model states: "+str(e))
        return []

def get_gem_model():
    """Automatically check which GEM model is present in the simulation"""
    models = get_available_models()
    
    if GEM_E2 in models:
        return GEM_E2
    elif GEM_E4 in models:
        return GEM_E4
    else:
        rospy.logwarn(f"Neither {GEM_E2} nor {GEM_E4} found in simulation. Defaulting to {GEM_E4}.")
        return GEM_E4

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

def set_position(x = 0, y = 0, z = 0, yaw = 0, vehicle_name = None):
    if vehicle_name is None:
        vehicle_name = get_gem_model()
    
    curr_state = getModelState(vehicle_name)
    new_state = ModelState()

    new_state.model_name = vehicle_name
    new_state.twist.linear.x = 0
    new_state.twist.linear.y = 0
    new_state.twist.linear.z = 0
    new_state.pose.position.x = x
    new_state.pose.position.y = y
    new_state.pose.position.z = z
   
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
    z_default = 0
    yaw_default = 3.14

    parser.add_argument('--x', type = float, help = 'x position of the vehicle.', default = x_default)
    parser.add_argument('--y', type = float, help = 'y position of the vehicle.', default = y_default)
    parser.add_argument('--z', type = float, help = 'z position of the vehicle.', default = z_default)
    parser.add_argument('--yaw', type = float, help = 'yaw of the vehicle.', default = yaw_default)
    parser.add_argument('--vehicle', type = str, help = 'vehicle name (overrides automatic detection)', default = None)

    argv = parser.parse_args()

    x = argv.x
    y = argv.y
    z = argv.z
    yaw = argv.yaw
    vehicle_name = argv.vehicle

    set_position(x = x, y = y, z = z, yaw = yaw, vehicle_name = vehicle_name)
