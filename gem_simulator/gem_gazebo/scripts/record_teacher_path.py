#!/usr/bin/env python3

import csv
import math
import os

import rospy
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


GEM_E2 = "gem_e2"
GEM_E4 = "gem_e4"


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TeacherPathRecorder:
    def __init__(self):
        self.vehicle_name = rospy.get_param("~vehicle_name", "")
        self.output_csv = os.path.expanduser(
            rospy.get_param("~output_csv", "~/teacher_paths/teacher_path.csv")
        )
        self.min_sample_distance = float(rospy.get_param("~min_sample_distance", 0.25))
        self.min_sample_heading = float(rospy.get_param("~min_sample_heading", 0.08))
        self.flush_every = int(rospy.get_param("~flush_every", 20))

        self.model_states = None
        self.vehicle_index = None
        self.samples = []
        self.cumulative_s = 0.0
        self.pending_writes = 0

        output_dir = os.path.dirname(self.output_csv)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        with open(self.output_csv, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["index", "s", "x", "y", "yaw"])

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)

    def model_states_callback(self, msg):
        self.model_states = msg
        self.vehicle_index = self.resolve_vehicle_index(msg)

    def resolve_vehicle_index(self, msg):
        if self.vehicle_name and self.vehicle_name in msg.name:
            return msg.name.index(self.vehicle_name)

        for candidate in (GEM_E4, GEM_E2):
            if candidate in msg.name:
                self.vehicle_name = candidate
                return msg.name.index(candidate)

        return None

    def get_current_pose(self):
        if self.model_states is None or self.vehicle_index is None:
            return None

        pose = self.model_states.pose[self.vehicle_index]
        x = float(pose.position.x)
        y = float(pose.position.y)
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return x, y, float(yaw)

    def should_record(self, x, y, yaw):
        if not self.samples:
            return True

        prev = self.samples[-1]
        distance = math.hypot(x - prev["x"], y - prev["y"])
        heading_delta = abs(normalize_angle(yaw - prev["yaw"]))
        return distance >= self.min_sample_distance or heading_delta >= self.min_sample_heading

    def append_sample(self, x, y, yaw):
        if self.samples:
            prev = self.samples[-1]
            self.cumulative_s += math.hypot(x - prev["x"], y - prev["y"])

        sample = {
            "index": len(self.samples),
            "s": self.cumulative_s,
            "x": x,
            "y": y,
            "yaw": yaw,
        }
        self.samples.append(sample)

        with open(self.output_csv, "a", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([sample["index"], sample["s"], sample["x"], sample["y"], sample["yaw"]])

        self.pending_writes += 1
        if self.pending_writes >= self.flush_every:
            self.pending_writes = 0
            rospy.loginfo(
                "Recorded %d path points. Latest: s=%.2f x=%.2f y=%.2f yaw=%.2f",
                len(self.samples),
                sample["s"],
                sample["x"],
                sample["y"],
                sample["yaw"],
            )

    def spin(self):
        rate = rospy.Rate(20.0)
        rospy.loginfo("Recording teacher path to %s", self.output_csv)
        while not rospy.is_shutdown():
            current_pose = self.get_current_pose()
            if current_pose is None:
                rospy.loginfo_throttle(2.0, "Waiting for /gazebo/model_states and GEM model")
                rate.sleep()
                continue

            x, y, yaw = current_pose
            if self.should_record(x, y, yaw):
                self.append_sample(x, y, yaw)

            rate.sleep()


def main():
    rospy.init_node("record_teacher_path")
    TeacherPathRecorder().spin()


if __name__ == "__main__":
    main()
