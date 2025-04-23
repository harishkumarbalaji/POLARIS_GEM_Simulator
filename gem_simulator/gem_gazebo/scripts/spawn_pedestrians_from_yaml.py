#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import textwrap
import yaml
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

# ─────────────────────────────────────────────────────────────────────────────
#  Collision-box [edit if required for you requirements]
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_COLLISIONS = {
    "LHipJoint_LeftUpLeg_collision":        {"scale": [0.01, 0.001, 0.001]},
    "LeftUpLeg_LeftLeg_collision":          {"scale": [ 8.0,  8.0,  1.0]},
    "LeftLeg_LeftFoot_collision":           {"scale": [ 8.0,  8.0,  1.0]},
    "LeftFoot_LeftToeBase_collision":       {"scale": [ 4.0,  4.0,  1.5]},

    "RHipJoint_RightUpLeg_collision":       {"scale": [0.01, 0.001, 0.001]},
    "RightUpLeg_RightLeg_collision":        {"scale": [ 8.0,  8.0,  1.0]},
    "RightLeg_RightFoot_collision":         {"scale": [ 8.0,  8.0,  1.0]},
    "RightFoot_RightToeBase_collision":     {"scale": [ 4.0,  4.0,  3.0]},

    "LowerBack_Spine_collision": {
        "scale": [12.0, 20.0, 5.0],
        "pose":  [0.05, 0, 0, 0, -0.2, 0]
    },

    "Spine_Spine1_collision":               {"scale": [0.01, 0.001, 0.001]},
    "Neck_Neck1_collision":                 {"scale": [0.01, 0.001, 0.001]},
    "Neck1_Head_collision":                 {"scale": [ 5.0,  5.0,  3.0]},

    "LeftShoulder_LeftArm_collision":       {"scale": [0.01, 0.001, 0.001]},
    "LeftArm_LeftForeArm_collision":        {"scale": [ 5.0,  5.0,  1.0]},
    "LeftForeArm_LeftHand_collision":       {"scale": [ 5.0,  5.0,  1.0]},
    "LeftFingerBase_LeftHandIndex1_collision":  {"scale": [4.0, 4.0, 3.0]},

    "RightShoulder_RightArm_collision":     {"scale": [0.01, 0.001, 0.001]},
    "RightArm_RightForeArm_collision":      {"scale": [ 5.0,  5.0,  1.0]},
    "RightForeArm_RightHand_collision":     {"scale": [ 5.0,  5.0,  1.0]},
    "RightFingerBase_RightHandIndex1_collision": {"scale": [4.0, 4.0, 3.0]},
}

# ─────────────────────────────────────────────────────────────────────────────
#  Default appearance & behaviour templates
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_SKIN = {"file": "walk.dae", "scale": 1.0}

DEFAULT_ANIMATION = {
    "name": "walking",
    "file": "walk.dae",
    "scale": 1.0,
    "interpolate_x": True,
}

DEFAULT_SCRIPT_FLAGS = {
    "loop": True,
    "delay_start": 0.0,
    "auto_start": True,
}

# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────
def collision_xml(collisions: dict) -> str:
    """Return the <scaling …/> XML for the collision plugin."""
    xml_lines = []
    for cname, cfg in collisions.items():
        scale = " ".join(map(str, cfg["scale"]))
        pose_attr = ""
        if "pose" in cfg:
            pose_attr = f'\n                pose="{" ".join(map(str, cfg["pose"]))}"'
        xml_lines.append(
            f'        <scaling collision="{cname}" scale="{scale}"{pose_attr}/>'
        )
    return "\n".join(xml_lines)


def make_actor_sdf(ped: dict) -> str:
    """Convert one pedestrian dict into an <actor>…</actor> block."""
    # Merge defaults with any overrides present in YAML
    skin = {**DEFAULT_SKIN, **ped.get("skin", {})}
    anim = {**DEFAULT_ANIMATION, **ped.get("animation", {})}

    trajectory = ped.get("trajectory")
    script_block = ped.get("script", {})
    if trajectory is None:
        trajectory = script_block.get("trajectory", [])
    if not trajectory:
        rospy.logwarn("Pedestrian '%s' has no trajectory; skipping.", ped["name"])
        return ""

    flags = {**DEFAULT_SCRIPT_FLAGS, **script_block}

    collisions = {**DEFAULT_COLLISIONS, **ped.get("collisions", {})}

    wp_xml = []
    for wp in trajectory:
        t, x, y, z, roll, pitch, yaw = wp
        wp_xml.append(
            f"""          <waypoint>
            <time>{t}</time>
            <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
          </waypoint>"""
        )

    return textwrap.dedent(f"""\
        <actor name="{ped['name']}">
          <skin>
            <filename>{skin['file']}</filename>
            <scale>{skin['scale']}</scale>
          </skin>

          <animation name="{anim['name']}">
            <filename>{anim['file']}</filename>
            <scale>{anim['scale']}</scale>
            <interpolate_x>{str(anim['interpolate_x']).lower()}</interpolate_x>
          </animation>

          <script>
            <loop>{str(flags['loop']).lower()}</loop>
            <delay_start>{flags['delay_start']:.6f}</delay_start>
            <auto_start>{str(flags['auto_start']).lower()}</auto_start>
            <trajectory id="0" type="walking">
{chr(10).join(wp_xml)}
            </trajectory>
          </script>

          <plugin name="{ped['name']}_collisions_plugin"
                  filename="libActorCollisionsPlugin.so">
{collision_xml(collisions)}
          </plugin>
        </actor>""")


# ─────────────────────────────────────────────────────────────────────────────
#  Main node
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    rospy.init_node("pedestrian_spawner")

    yaml_path = rospy.get_param("~yaml_path")
    rospy.loginfo("Reading pedestrian data from %s", yaml_path)
    with open(yaml_path, "r") as f:
        scene = yaml.safe_load(f)

    pedestrians = scene.get("pedestrians", [])
    if not pedestrians:
        rospy.logwarn("No 'pedestrians' list found in YAML — nothing to spawn.")
        return

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for ped in pedestrians:
        actor_xml = make_actor_sdf(ped)
        if not actor_xml:
            continue  # skip if trajectory was missing
        sdf = f'<?xml version="1.0"?><sdf version="1.7">{actor_xml}</sdf>'
        try:
            resp = spawn_srv(ped["name"], sdf, "/", Pose(), "world")
            if resp.success:
                rospy.loginfo("Spawned pedestrian '%s'", ped["name"])
            else:
                rospy.logwarn("Failed to spawn '%s': %s",
                              ped["name"], resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed for '%s': %s", ped["name"], e)

    rospy.loginfo("All pedestrians spawned.")


if __name__ == "__main__":
    main()
