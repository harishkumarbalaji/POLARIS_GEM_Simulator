#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spawns two kinds of agents from a YAML file:

```
agents:
  - name: pedestrian1               # actor will have no motion param
    source: {uri: ...}
    trajectory: [ [t,x,y,z,r,p,yaw], ... ]

  - name: bicycle1                  # rigid will have a motion param
    motion: rigid
    source: {uri: ...}
    trajectory: [ [t,x,y,z,r,p,yaw], ... ]
```

* **Actor** → uses Gazebo `<actor>` element (animated internally).
* **Rigid** → dynamic include, per-link gravity disabled, moved kinematically via `/gazebo/set_model_state` in a looping trajectory.
"""

import sys, yaml, threading, time
from typing import List

import rospy
from gazebo_msgs.srv import (
    SpawnModel,
    GetModelProperties,
    GetLinkProperties,
    SetLinkProperties,
    SetModelState,
)
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import quaternion_from_euler

# ───────── defaults ─────────
DEFAULT_SKIN   = {"file": "walk.dae", "scale": 1.0}
DEFAULT_ANIM   = {"name": "walking", "file": "walk.dae", "scale": 1.0, "interpolate_x": True}
DEFAULT_SCRIPT = {"loop": True, "delay_start": 0.0, "auto_start": True}

# ───────── helpers functions ─────────

def build_include_sdf(name: str, uri: str) -> str:
    return f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.7\">  
  <model name=\"{name}\">  
    <static>false</static>  
    <include><uri>{uri}</uri></include>  
  </model>
</sdf>"""


def build_actor_sdf(agent: dict) -> str:
    skin  = {**DEFAULT_SKIN,   **agent.get("skin", {})}
    anim  = {**DEFAULT_ANIM,   **agent.get("animation", {})}
    flags = {**DEFAULT_SCRIPT, **agent.get("script", {})}
    wps   = agent.get("trajectory", [])
    if not wps:
        rospy.logwarn("Actor '%s' missing trajectory", agent["name"])
        return ""

    wp_xml = "\n".join(
        f"<waypoint><time>{t}</time><pose>{x} {y} {z} {r} {p} {y_}</pose></waypoint>"
        for t, x, y, z, r, p, y_ in wps
    )

    return f"""<?xml version=\"1.0\"?>
<sdf version=\"1.7\">  
  <actor name=\"{agent['name']}\">  
    <skin><filename>{skin['file']}</filename><scale>{skin['scale']}</scale></skin>  
    <animation name=\"{anim['name']}\">  
      <filename>{anim['file']}</filename>  
      <scale>{anim['scale']}</scale>  
      <interpolate_x>{str(anim['interpolate_x']).lower()}</interpolate_x>  
    </animation>  
    <script>  
      <loop>{str(flags['loop']).lower()}</loop>  
      <delay_start>{flags['delay_start']}</delay_start>  
      <auto_start>{str(flags['auto_start']).lower()}</auto_start>  
      <trajectory id=\"0\" type=\"walking\">  
        {wp_xml}  
      </trajectory>  
    </script>  
  </actor>
</sdf>"""


def to_pose(xyz, rpy) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
    q = quaternion_from_euler(*map(float, rpy))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose

# ───────── physics helper functions ─────────

def disable_gravity_for_links(model_name: str):
    try:
        gm = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        gl = rospy.ServiceProxy("/gazebo/get_link_properties",  GetLinkProperties)
        sl = rospy.ServiceProxy("/gazebo/set_link_properties",  SetLinkProperties)
        rospy.wait_for_service("/gazebo/get_model_properties", timeout=5)
        links = gm(model_name).body_names
        for link in links:
            full = f"{model_name}::{link}"
            lp   = gl(full)
            sl(
                link_name    = full,
                com          = lp.com if isinstance(lp.com, Vector3) else Vector3(),
                gravity_mode = False,
                mass         = lp.mass,
                ixx=lp.ixx, ixy=lp.ixy, ixz=lp.ixz,
                iyy=lp.iyy, iyz=lp.iyz,
                izz=lp.izz,
            )
    except Exception as e:
        rospy.logwarn("disable_gravity_for_links failed: %s", e)

# ───────── rigid motion ─────────

def _rigid_motion_loop(set_state_srv, model: str, traj: List[list], rate_hz: int = 50):
    rate = rospy.Rate(rate_hz)
    # loop trajectory indefinitely so the objects move indefinetly
    while not rospy.is_shutdown():
        for i in range(1, len(traj)):
            t0, *p0 = traj[i - 1]
            t1, *p1 = traj[i]
            duration = max(0.0001, t1 - t0)
            steps    = max(2, int(duration * rate_hz))
            for s in range(steps):
                f = s / float(steps)
                xyz = [p0[j] + f * (p1[j] - p0[j]) for j in range(3)]
                rpy = [p0[j+3] + f * (p1[j+3] - p0[j+3]) for j in range(3)]
                state = ModelState(model_name=model, pose=to_pose(xyz, rpy), twist=Twist(), reference_frame="world")
                try:
                    set_state_srv(state)
                except rospy.ServiceException as err:
                    rospy.logwarn_throttle(5, "set_model_state failed: %s", err)
                rate.sleep()


def start_rigid_motion(name: str, traj: List[list]):
    rospy.wait_for_service("/gazebo/set_model_state")
    set_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    threading.Thread(target=_rigid_motion_loop, args=(set_srv, name, traj), daemon=True).start()

# ───────── spawn helper function─────────

def spawn_model(spawn_srv, name: str, sdf: str, pose: Pose = Pose()):
    if spawn_srv(name, sdf, "", pose, "world").success:
        rospy.loginfo("Spawned %s", name)
        return True
    rospy.logwarn("Failed to spawn %s", name)
    return False

# ───────── main ─────────

def main():
    rospy.init_node("spawn_agents")
    yaml_path = rospy.get_param("~yaml_path", "")
    if not yaml_path:
        rospy.logfatal("Missing ~yaml_path parameter")
        sys.exit(1)

    agents = (yaml.safe_load(open(yaml_path)) or {}).get("agents", [])
    if not agents:
        rospy.logfatal("No 'agents' in YAML")
        sys.exit(1)

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for ag in agents:
        name   = ag["name"]
        uri    = ag["source"]["uri"]
        motion = ag.get("motion", "actor")

        try:
            if motion == "rigid":
                pose = to_pose(ag["trajectory"][0][1:4], ag["trajectory"][0][4:])
                if not spawn_model(spawn_srv, name, build_include_sdf(name, uri), pose):
                    continue
                disable_gravity_for_links(name)
                start_rigid_motion(name, ag["trajectory"])

            else:
                sdf = build_actor_sdf(ag)
                if sdf:
                    spawn_model(spawn_srv, name, sdf)

        except Exception as err:
            rospy.logerr("Error processing %s: %s", name, err)

    rospy.loginfo("All agents spawned; entering spin() to keep node alive.")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
