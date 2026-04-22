#!/usr/bin/env python3

import threading

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraStopController:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/oak/rgb/image_raw")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/ackermann_cmd")
        self.speed = float(rospy.get_param("~speed", 2.0))
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.init_frames = int(rospy.get_param("~init_frames", 20))
        self.stop_threshold = float(rospy.get_param("~stop_threshold", 22.0))
        self.resume_threshold = float(rospy.get_param("~resume_threshold", 16.0))
        self.roi_x_min = float(rospy.get_param("~roi_x_min", 0.35))
        self.roi_x_max = float(rospy.get_param("~roi_x_max", 0.65))
        self.roi_y_min = float(rospy.get_param("~roi_y_min", 0.45))
        self.roi_y_max = float(rospy.get_param("~roi_y_max", 0.85))

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.cmd_topic, AckermannDrive, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        self.lock = threading.Lock()
        self.baseline_roi = None
        self.baseline_accumulator = None
        self.frames_seen = 0
        self.obstacle_detected = False
        self.last_score = 0.0

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert image: %s", exc)
            return

        gray = self.to_grayscale(image)
        roi = self.extract_roi(gray)
        if roi.size == 0:
            rospy.logwarn_throttle(5.0, "Empty ROI from image topic %s", self.image_topic)
            return

        roi = roi.astype(np.float32)

        with self.lock:
            if self.frames_seen < self.init_frames:
                if self.baseline_accumulator is None:
                    self.baseline_accumulator = np.zeros_like(roi, dtype=np.float32)
                self.baseline_accumulator += roi
                self.frames_seen += 1
                if self.frames_seen == self.init_frames:
                    self.baseline_roi = self.baseline_accumulator / float(self.init_frames)
                    rospy.loginfo(
                        "Camera baseline initialized from %d frames on %s",
                        self.init_frames,
                        self.image_topic,
                    )
                return

            diff = np.abs(roi - self.baseline_roi)
            score = float(diff.mean())
            self.last_score = score

            if self.obstacle_detected:
                if score < self.resume_threshold:
                    self.obstacle_detected = False
                    rospy.loginfo("Obstacle cleared, resuming. score=%.2f", score)
            else:
                if score > self.stop_threshold:
                    self.obstacle_detected = True
                    rospy.loginfo("Obstacle detected, stopping. score=%.2f", score)

    @staticmethod
    def to_grayscale(image):
        # BGR -> grayscale without requiring OpenCV at runtime.
        b = image[:, :, 0].astype(np.float32)
        g = image[:, :, 1].astype(np.float32)
        r = image[:, :, 2].astype(np.float32)
        return 0.114 * b + 0.587 * g + 0.299 * r

    def extract_roi(self, gray):
        height, width = gray.shape
        x0 = max(0, min(width, int(width * self.roi_x_min)))
        x1 = max(0, min(width, int(width * self.roi_x_max)))
        y0 = max(0, min(height, int(height * self.roi_y_min)))
        y1 = max(0, min(height, int(height * self.roi_y_max)))
        return gray[y0:y1, x0:x1]

    def publish_cmd(self, speed):
        msg = AckermannDrive()
        msg.steering_angle = 0.0
        msg.speed = speed
        self.pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                ready = self.baseline_roi is not None
                obstacle_detected = self.obstacle_detected
                score = self.last_score

            if not ready:
                self.publish_cmd(0.0)
                rospy.loginfo_throttle(2.0, "Waiting for camera baseline on %s", self.image_topic)
            elif obstacle_detected:
                self.publish_cmd(0.0)
                rospy.loginfo_throttle(1.0, "Camera stop active. score=%.2f", score)
            else:
                self.publish_cmd(self.speed)
                rospy.loginfo_throttle(1.0, "Driving straight. score=%.2f", score)

            rate.sleep()

        self.publish_cmd(0.0)


def main():
    rospy.init_node("drive_straight_with_camera_stop")
    controller = CameraStopController()
    controller.spin()


if __name__ == "__main__":
    main()
