#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import rospy
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion


GEM_E2 = "gem_e2"
GEM_E4 = "gem_e4"


class PilotNetCollector:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/oak/rgb/image_raw")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/ackermann_cmd")
        self.output_root = os.path.expanduser(
            rospy.get_param("~output_root", "~/pilotnet_data")
        )
        self.session_name = rospy.get_param(
            "~session_name", datetime.now().strftime("session_%Y%m%d_%H%M%S")
        )
        self.vehicle_name = rospy.get_param("~vehicle_name", "")
        self.image_format = rospy.get_param("~image_format", "jpg")
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 95))
        self.min_sample_interval = float(rospy.get_param("~min_sample_interval", 0.10))
        self.min_abs_speed_to_record = float(rospy.get_param("~min_abs_speed_to_record", 0.05))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.5))

        self.bridge = CvBridge()
        self.latest_cmd = None
        self.latest_cmd_stamp = None
        self.latest_pose = None
        self.latest_pose_stamp = None
        self.latest_model_states = None
        self.vehicle_index = None
        self.frame_counter = 0
        self.last_saved_stamp = None

        self.session_dir = os.path.join(self.output_root, self.session_name)
        self.images_dir = os.path.join(self.session_dir, "images")
        os.makedirs(self.images_dir, exist_ok=True)

        self.metadata_path = os.path.join(self.session_dir, "metadata.csv")
        self.metadata_file = open(self.metadata_path, "w", newline="")
        self.metadata_writer = csv.writer(self.metadata_file)
        self.metadata_writer.writerow(
            [
                "frame_id",
                "timestamp",
                "image_path",
                "steering",
                "speed",
                "x",
                "y",
                "yaw",
                "vehicle_name",
            ]
        )
        self.metadata_file.flush()

        rospy.Subscriber(self.cmd_topic, AckermannDrive, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        rospy.on_shutdown(self.shutdown)

    def cmd_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_stamp = rospy.Time.now()

    def model_states_callback(self, msg):
        self.latest_model_states = msg
        self.vehicle_index = self.resolve_vehicle_index(msg)
        if self.vehicle_index is None:
            return

        pose = msg.pose[self.vehicle_index]
        x = float(pose.position.x)
        y = float(pose.position.y)
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.latest_pose = (x, y, float(yaw))
        self.latest_pose_stamp = rospy.Time.now()

    def resolve_vehicle_index(self, msg):
        if self.vehicle_name and self.vehicle_name in msg.name:
            return msg.name.index(self.vehicle_name)

        for candidate in (GEM_E4, GEM_E2):
            if candidate in msg.name:
                self.vehicle_name = candidate
                return msg.name.index(candidate)

        return None

    def image_callback(self, msg):
        now = rospy.Time.now()

        if self.latest_cmd is None or self.latest_pose is None:
            rospy.loginfo_throttle(2.0, "Waiting for command and pose before recording")
            return

        if self.latest_cmd_stamp is None or (now - self.latest_cmd_stamp).to_sec() > self.command_timeout:
            rospy.loginfo_throttle(2.0, "Skipping frame: teacher command is stale")
            return

        if self.latest_pose_stamp is None or (now - self.latest_pose_stamp).to_sec() > self.pose_timeout:
            rospy.loginfo_throttle(2.0, "Skipping frame: vehicle pose is stale")
            return

        if abs(float(self.latest_cmd.speed)) < self.min_abs_speed_to_record:
            return

        if self.last_saved_stamp is not None:
            if (now - self.last_saved_stamp).to_sec() < self.min_sample_interval:
                return

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert image for dataset: %s", exc)
            return

        self.frame_counter += 1
        frame_id = f"{self.frame_counter:06d}"
        image_filename = f"{frame_id}.{self.image_format}"
        image_path = os.path.join(self.images_dir, image_filename)
        relative_image_path = os.path.join("images", image_filename)

        pil_image = PILImage.fromarray(rgb)
        if self.image_format.lower() in ("jpg", "jpeg"):
            pil_image.save(image_path, quality=self.jpeg_quality)
        else:
            pil_image.save(image_path)

        x, y, yaw = self.latest_pose
        self.metadata_writer.writerow(
            [
                frame_id,
                f"{now.to_sec():.6f}",
                relative_image_path,
                f"{float(self.latest_cmd.steering_angle):.6f}",
                f"{float(self.latest_cmd.speed):.6f}",
                f"{x:.6f}",
                f"{y:.6f}",
                f"{yaw:.6f}",
                self.vehicle_name,
            ]
        )
        self.metadata_file.flush()
        self.last_saved_stamp = now

        rospy.loginfo_throttle(
            1.0,
            "Collected %d frames in %s",
            self.frame_counter,
            self.session_dir,
        )

    def shutdown(self):
        try:
            self.metadata_file.close()
        except Exception:
            pass


def main():
    rospy.init_node("collect_pilotnet_data")
    collector = PilotNetCollector()
    rospy.loginfo("Saving PilotNet dataset to %s", collector.session_dir)
    rospy.spin()


if __name__ == "__main__":
    main()
