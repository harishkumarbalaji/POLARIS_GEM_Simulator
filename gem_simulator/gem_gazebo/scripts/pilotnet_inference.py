#!/usr/bin/env python3

import os

import numpy as np
from PIL import Image as PILImage
import rospy
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import torch
import torch.nn as nn


class PilotNet(nn.Module):
    """Minimal PilotNet-style steering regressor."""

    def __init__(self, in_channels=3, dropout=0.1):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(in_channels, 24, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3, stride=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64, 100),
            nn.ReLU(inplace=True),
            nn.Dropout(p=dropout),
            nn.Linear(100, 50),
            nn.ReLU(inplace=True),
            nn.Dropout(p=dropout),
            nn.Linear(50, 10),
            nn.ReLU(inplace=True),
            nn.Linear(10, 1),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.regressor(x)
        return x.squeeze(-1)


class PilotNetInferenceNode:
    def __init__(self):
        default_checkpoint = (
            "/home/{user}/host/Documents/UIUC-courses/CS588AV/Project/"
            "pilotnet_runs/run_001/best_model.pt"
        ).format(user=os.environ.get("USER", "yuwei"))

        self.image_topic = rospy.get_param("~image_topic", "/oak/rgb/image_raw")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/ackermann_cmd")
        self.checkpoint_path = os.path.expanduser(
            rospy.get_param("~checkpoint", default_checkpoint)
        )
        self.speed = float(rospy.get_param("~speed", 0.35))
        self.max_steering = float(rospy.get_param("~max_steering", 0.55))
        self.steering_scale = float(rospy.get_param("~steering_scale", 1.0))
        self.steering_smoothing = float(rospy.get_param("~steering_smoothing", 0.25))

        self.bridge = CvBridge()
        self.prev_steering = None

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model, self.model_args = self.load_model(self.checkpoint_path)

        # These defaults mirror training and can be overridden via ROS params.
        self.image_width = int(
            rospy.get_param("~image_width", self.model_args.get("image_width", 200))
        )
        self.image_height = int(
            rospy.get_param("~image_height", self.model_args.get("image_height", 66))
        )
        self.crop_top_ratio = float(
            rospy.get_param("~crop_top_ratio", self.model_args.get("crop_top_ratio", 0.35))
        )
        self.crop_bottom_ratio = float(
            rospy.get_param("~crop_bottom_ratio", self.model_args.get("crop_bottom_ratio", 0.10))
        )

        self.pub = rospy.Publisher(self.cmd_topic, AckermannDrive, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        rospy.on_shutdown(self.publish_stop)

    def load_model(self, checkpoint_path):
        if not os.path.exists(checkpoint_path):
            raise RuntimeError("Checkpoint not found: %s" % checkpoint_path)

        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        model_args = checkpoint.get("args", {})
        dropout = float(model_args.get("dropout", 0.1))

        model = PilotNet(dropout=dropout).to(self.device)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.eval()
        return model, model_args

    def preprocess_image(self, rgb):
        image = PILImage.fromarray(rgb)
        width, height = image.size
        top = int(height * self.crop_top_ratio)
        bottom = int(height * (1.0 - self.crop_bottom_ratio))
        bottom = max(bottom, top + 1)

        image = image.crop((0, top, width, bottom))
        image = image.resize((self.image_width, self.image_height), PILImage.BILINEAR)
        image = np.asarray(image, dtype=np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))
        tensor = torch.from_numpy(image).unsqueeze(0).to(self.device)
        return tensor

    def smooth_steering(self, steering):
        if self.prev_steering is None:
            self.prev_steering = steering
            return steering

        alpha = min(max(self.steering_smoothing, 0.0), 1.0)
        steering = alpha * steering + (1.0 - alpha) * self.prev_steering
        self.prev_steering = steering
        return steering

    def publish_stop(self):
        self.pub.publish(AckermannDrive())

    def image_callback(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert image for PilotNet inference: %s", exc)
            return

        inputs = self.preprocess_image(rgb)
        with torch.no_grad():
            steering = float(self.model(inputs).item())

        steering *= self.steering_scale
        steering = max(-self.max_steering, min(steering, self.max_steering))
        steering = self.smooth_steering(steering)

        cmd = AckermannDrive()
        cmd.speed = self.speed
        cmd.steering_angle = steering
        self.pub.publish(cmd)

        rospy.loginfo_throttle(
            1.0,
            "PilotNet inference: steering=%.3f speed=%.2f device=%s",
            steering,
            self.speed,
            self.device.type,
        )


def main():
    rospy.init_node("pilotnet_inference")
    node = PilotNetInferenceNode()
    rospy.loginfo("Loaded PilotNet checkpoint from %s", node.checkpoint_path)
    rospy.spin()


if __name__ == "__main__":
    main()
