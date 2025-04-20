#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spawn Construction‑Cone models at runtime, using positions
read from a YAML file
"""
import yaml
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler


def make_sdf(uri: str, name: str) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <include>
      <uri>{uri}</uri>
    </include>
  </model>
</sdf>
"""


def main():
    rospy.init_node("cone_spawner")

    # Parameters passed from the launch file
    yaml_path = rospy.get_param("~yaml_path")
    model_uri = rospy.get_param("~model_uri")

    rospy.loginfo("Reading cone list from %s", yaml_path)
    with open(yaml_path, "r") as f:
        cones = yaml.safe_load(f)["cones"]

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for cone in cones:
        # --- Build the initial pose ----------------------------------------
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, cone["xyz"])
        qx, qy, qz, qw = quaternion_from_euler(*map(float, cone["rpy"]))
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # --- Build the SDF for this cone -----------------------------------
        sdf_xml = make_sdf(model_uri, cone["name"])

        # --- Call the spawn service ----------------------------------------
        try:
            resp = spawn_srv(cone["name"], sdf_xml, "/", pose, "world")
            if resp.success:
                rospy.loginfo("Spawned %s", cone["name"])
            else:
                rospy.logwarn("Failed to spawn %s: %s",
                              cone["name"], resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    rospy.loginfo("Cone‑spawner finished.")


if __name__ == "__main__":
    main()
