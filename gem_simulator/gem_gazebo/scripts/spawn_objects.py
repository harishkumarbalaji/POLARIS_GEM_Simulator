#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spawn_objects.py
────────────────
Generic spawner for *static* objects declared in a YAML scene file.

YAML schema  
It can take two types its either fuel or mesh file
-----------------------------------------------------
objects:
  - name: cone_1
    source: {type: fuel, uri: "https://fuel.gazebosim.org/.../Construction%20Cone"}
    xyz: [x, y, z]
    rpy: [roll, pitch, yaw]

  - name: stop_sign
    source: {type: mesh, uri: "model://stop_sign/meshes/Stop_Sign.stl", scale: [1.0, 1.0, 1.0]}
    xyz: [...]
    rpy: [...]
"""
import sys
import yaml
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from typing import List 


# ─────────────────────────── helper function ────────────────────────────
def build_sdf(name: str, src_type: str, uri: str, scale=None, static: bool = True) -> str:
    """
    Return an SDF <model> string based on the asset source:
      • mesh  -> inlines the STL/DAE
      • anything else (sdf / urdf / fuel) -> <include><uri>...
    """
    src_type = (src_type or "").lower()
    
    # Use default scale [1,1,1] if not provided
    if scale is None:
        scale = [1.0, 1.0, 1.0]
    
    # Make sure scale is a list of 3 floats
    scale = list(map(float, scale[:3])) if len(scale) >= 3 else [1.0, 1.0, 1.0]
    scale_str = f"<scale>{scale[0]} {scale[1]} {scale[2]}</scale>"

    if src_type == "mesh":
        return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <static>{str(static).lower()}</static>
    <link name="link">
      <visual name="visual">
        <geometry><mesh><uri>{uri}</uri>{scale_str}</mesh></geometry>
      </visual>
      <collision name="collision">
        <geometry><mesh><uri>{uri}</uri>{scale_str}</mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    # default path: wrap whatever it is in <include>
    return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <static>{str(static).lower()}</static>
    <include>
      <uri>{uri}</uri>
    </include>
  </model>
</sdf>
"""


def yaml_objects(scene_yaml: str) -> List[dict]:
    """Load YAML and return the list under `objects` (or empty list)."""
    with open(scene_yaml, "r") as f:
        scene = yaml.safe_load(f) or {}
    objs = scene.get("objects", [])
    if not objs:
        rospy.logwarn("No 'objects' key found in YAML or list is empty.")
    return objs


def to_pose(xyz, rpy) -> Pose:
    """Convert [x,y,z] & [r,p,y] lists into a geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
    qx, qy, qz, qw = quaternion_from_euler(*map(float, rpy))
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def main() -> None:
    rospy.init_node("spawn_objects")
    yaml_path = rospy.get_param("~yaml_path", default="")
    if not yaml_path:
        rospy.logfatal("Parameter '~yaml_path' not supplied.")
        sys.exit(1)

    objects = yaml_objects(yaml_path)
    if not objects:
        rospy.logfatal("Nothing to spawn — exiting.")
        sys.exit(1)

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    rospy.loginfo("Spawning %d static objects …", len(objects))

    for obj in objects:
        name   = obj.get("name")
        xyz    = obj.get("xyz", [0, 0, 0])
        rpy    = obj.get("rpy", [0, 0, 0])
        source = obj.get("source", {})
        s_type = source.get("type", "sdf")   # default to include
        uri    = source.get("uri", "")
        scale  = source.get("scale", [1.0, 1.0, 1.0])  # Get scale or use default

        if not name or not uri:
            rospy.logwarn("Skipping invalid object entry: %s", obj)
            continue

        sdf_xml = build_sdf(name, s_type, uri, scale, static=True)
        pose    = to_pose(xyz, rpy)

        try:
            resp = spawn_srv(name, sdf_xml, "/", pose, "world")
            if resp.success:
                rospy.loginfo("Spawned object '%s'", name)
            else:
                rospy.logwarn("Failed to spawn '%s': %s",
                              name, resp.status_message)
        except rospy.ServiceException as exc:
            rospy.logerr("Service call failed for '%s': %s", name, exc)

    rospy.loginfo("spawn_objects.py finished.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
