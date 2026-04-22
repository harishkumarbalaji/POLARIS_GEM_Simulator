#!/usr/bin/env python3

import csv
import math
import os

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


GEM_E2 = "gem_e2"
GEM_E4 = "gem_e4"


class TeacherPathFollower:
    def __init__(self):
        self.vehicle_name = rospy.get_param("~vehicle_name", "")
        self.path_csv = os.path.expanduser(rospy.get_param("~path_csv", "~/teacher_paths/teacher_path.csv"))
        self.speed = float(rospy.get_param("~speed", 0.5))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.lookahead = float(rospy.get_param("~lookahead", 2.5))
        self.wheelbase = float(rospy.get_param("~wheelbase", 1.75))
        self.max_steering = float(rospy.get_param("~max_steering", 0.55))
        self.loop_path = bool(rospy.get_param("~loop_path", True))

        self.model_states = None
        self.vehicle_index = None
        self.path = self.load_path(self.path_csv)
        self.path_xs = np.array([point["x"] for point in self.path], dtype=np.float32)
        self.path_ys = np.array([point["y"] for point in self.path], dtype=np.float32)
        self.pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=1)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)

    def load_path(self, path_csv):
        if not os.path.exists(path_csv):
            raise RuntimeError("Path CSV not found: %s" % path_csv)

        rows = []
        with open(path_csv, newline="") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                rows.append(
                    {
                        "index": int(row["index"]),
                        "s": float(row["s"]),
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "yaw": float(row["yaw"]),
                    }
                )

        if len(rows) < 2:
            raise RuntimeError("Need at least 2 waypoints in path CSV: %s" % path_csv)
        return rows

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

    def find_nearest_index(self, x, y):
        distances = (self.path_xs - x) ** 2 + (self.path_ys - y) ** 2
        return int(np.argmin(distances))

    def find_lookahead_index(self, nearest_index):
        target_index = nearest_index
        path_length = len(self.path)
        accumulated = 0.0

        while accumulated < self.lookahead:
            next_index = target_index + 1
            if next_index >= path_length:
                if not self.loop_path:
                    return path_length - 1
                next_index = 0

            p0 = self.path[target_index]
            p1 = self.path[next_index]
            accumulated += math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
            target_index = next_index

            if target_index == nearest_index:
                break

        return target_index

    def compute_local_target(self, current_pose, target_point):
        x, y, yaw = current_pose
        dx = target_point["x"] - x
        dy = target_point["y"] - y

        x_local = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_local = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return x_local, y_local

    def publish_cmd(self, steering):
        msg = AckermannDrive()
        msg.speed = self.speed
        msg.steering_angle = steering
        self.pub.publish(msg)

    def publish_stop(self):
        self.pub.publish(AckermannDrive())

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Following teacher path from %s", self.path_csv)

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose()
            if current_pose is None:
                rospy.loginfo_throttle(2.0, "Waiting for /gazebo/model_states and GEM model")
                rate.sleep()
                continue

            nearest_index = self.find_nearest_index(current_pose[0], current_pose[1])
            lookahead_index = self.find_lookahead_index(nearest_index)
            target_point = self.path[lookahead_index]
            x_local, y_local = self.compute_local_target(current_pose, target_point)

            attempts = 0
            while x_local <= 0.0 and attempts < len(self.path):
                lookahead_index = self.find_lookahead_index(lookahead_index)
                target_point = self.path[lookahead_index]
                x_local, y_local = self.compute_local_target(current_pose, target_point)
                attempts += 1

            ld2 = max(x_local * x_local + y_local * y_local, 1e-6)
            steering = math.atan2(2.0 * self.wheelbase * y_local, ld2)
            steering = max(-self.max_steering, min(steering, self.max_steering))

            self.publish_cmd(steering)
            rospy.loginfo_throttle(
                1.0,
                "Teacher follow: nearest=%d target=%d x_local=%.2f y_local=%.2f steering=%.3f",
                nearest_index,
                lookahead_index,
                x_local,
                y_local,
                steering,
            )
            rate.sleep()

        self.publish_stop()


def main():
    rospy.init_node("teacher_path_follow")
    TeacherPathFollower().spin()


if __name__ == "__main__":
    main()
