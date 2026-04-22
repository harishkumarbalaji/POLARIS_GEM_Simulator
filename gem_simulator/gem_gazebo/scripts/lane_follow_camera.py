#!/usr/bin/env python3

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class LaneFollower:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/oak/rgb/image_raw")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/ackermann_cmd")
        self.speed = float(rospy.get_param("~speed", 1.0))
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.roi_y_min = float(rospy.get_param("~roi_y_min", 0.55))
        self.roi_y_max = float(rospy.get_param("~roi_y_max", 0.90))
        self.min_lane_strength = float(rospy.get_param("~min_lane_strength", 5000.0))
        self.steering_gain = float(rospy.get_param("~steering_gain", 0.9))
        self.max_steering = float(rospy.get_param("~max_steering", 0.35))
        self.min_white_intensity = float(rospy.get_param("~min_white_intensity", 185.0))
        self.max_channel_spread = float(rospy.get_param("~max_channel_spread", 35.0))
        self.scan_rows = int(rospy.get_param("~scan_rows", 12))
        self.min_row_strength = float(rospy.get_param("~min_row_strength", 20.0))
        self.lookahead_row_ratio = float(rospy.get_param("~lookahead_row_ratio", 0.35))
        self.stop_on_lane_loss = bool(rospy.get_param("~stop_on_lane_loss", True))
        self.follow_mode = rospy.get_param("~follow_mode", "center")
        self.right_boundary_offset_ratio = float(
            rospy.get_param("~right_boundary_offset_ratio", 0.18)
        )

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.cmd_topic, AckermannDrive, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        self.latest_error = 0.0
        self.latest_strength = 0.0
        self.have_lane = False
        self.rows_used = 0

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert lane image: %s", exc)
            return

        roi = self.extract_roi(image)
        if roi.size == 0:
            self.have_lane = False
            return

        lane_mask = self.extract_white_lane_mask(roi)
        histogram = lane_mask.sum(axis=0).astype(np.float32)
        lane_strength = float(histogram.sum())
        self.latest_strength = lane_strength

        if lane_strength < self.min_lane_strength:
            self.have_lane = False
            return

        row_measurements = self.extract_row_measurements(lane_mask)
        self.rows_used = len(row_measurements)
        if not row_measurements:
            self.have_lane = False
            return

        width = lane_mask.shape[1]
        lookahead_index = min(
            len(row_measurements) - 1,
            max(0, int(len(row_measurements) * self.lookahead_row_ratio)),
        )
        target_x = self.select_target_x(row_measurements[lookahead_index], width)
        if target_x is None:
            self.have_lane = False
            return

        image_center_x = width / 2.0
        error_pixels = target_x - image_center_x
        self.latest_error = error_pixels / image_center_x
        self.have_lane = True

    def extract_roi(self, image):
        height, width = image.shape[:2]
        y0 = max(0, min(height, int(height * self.roi_y_min)))
        y1 = max(0, min(height, int(height * self.roi_y_max)))
        return image[y0:y1, :width, :]

    def extract_white_lane_mask(self, image):
        # This detector is intentionally tuned for white lane markings:
        # keep bright pixels whose B/G/R channels are all similar, which
        # rejects colored regions better than a grayscale threshold alone.
        b = image[:, :, 0].astype(np.float32)
        g = image[:, :, 1].astype(np.float32)
        r = image[:, :, 2].astype(np.float32)

        max_channel = np.maximum(np.maximum(b, g), r)
        min_channel = np.minimum(np.minimum(b, g), r)
        white_mask = (max_channel >= self.min_white_intensity) & (
            (max_channel - min_channel) <= self.max_channel_spread
        )
        return white_mask

    @staticmethod
    def weighted_center(values, offset):
        indices = np.arange(values.shape[0], dtype=np.float32)
        total = values.sum()
        if total <= 0.0:
            return float(offset)
        return float(offset + (indices * values).sum() / total)

    def extract_row_measurements(self, lane_mask):
        # For curved white lanes, estimate a lane center separately on several
        # horizontal slices and steer toward a lookahead slice instead of using
        # one collapsed histogram over the entire ROI.
        height, width = lane_mask.shape
        row_edges = np.linspace(0, height, self.scan_rows + 1, dtype=int)
        measurements = []

        for idx in range(self.scan_rows - 1, -1, -1):
            y0 = row_edges[idx]
            y1 = row_edges[idx + 1]
            band = lane_mask[y0:y1, :]
            if band.size == 0:
                continue

            histogram = band.sum(axis=0).astype(np.float32)
            if float(histogram.sum()) < self.min_row_strength:
                continue

            midpoint = width // 2
            left = histogram[:midpoint]
            right = histogram[midpoint:]
            left_strength = float(left.sum())
            right_strength = float(right.sum())
            left_x = None
            right_x = None

            if left_strength > 0.0 and right_strength > 0.0:
                left_x = self.weighted_center(left, 0)
                right_x = self.weighted_center(right, midpoint)
                center_x = 0.5 * (left_x + right_x)
            elif right_strength > 0.0:
                right_x = self.weighted_center(right, midpoint)
                center_x = right_x
            elif left_strength > 0.0:
                left_x = self.weighted_center(left, 0)
                center_x = left_x
            else:
                center_x = self.weighted_center(histogram, 0)

            measurements.append(
                {
                    "center_x": center_x,
                    "left_x": left_x,
                    "right_x": right_x,
                    "left_strength": left_strength,
                    "right_strength": right_strength,
                }
            )

        measurements.reverse()
        return measurements

    def select_target_x(self, measurement, width):
        if self.follow_mode == "right_boundary":
            right_x = measurement["right_x"]
            if right_x is None:
                return None

            # This mode is intended for circular or strongly curved lanes:
            # stay a fixed offset to the left of the detected right white line
            # instead of aiming for the full lane center.
            offset_pixels = self.right_boundary_offset_ratio * width
            return right_x - offset_pixels

        return measurement["center_x"]

    def publish_cmd(self, speed, steering):
        msg = AckermannDrive()
        msg.speed = speed
        msg.steering_angle = steering
        self.pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self.have_lane:
                speed = 0.0 if self.stop_on_lane_loss else self.speed * 0.5
                self.publish_cmd(speed, 0.0)
                rospy.loginfo_throttle(
                    1.0,
                    "White lane not found. strength=%.1f threshold=%.1f rows=%d mode=%s",
                    self.latest_strength,
                    self.min_lane_strength,
                    self.rows_used,
                    self.follow_mode,
                )
            else:
                steering = -self.steering_gain * self.latest_error
                steering = max(-self.max_steering, min(steering, self.max_steering))
                self.publish_cmd(self.speed, steering)
                rospy.loginfo_throttle(
                    1.0,
                    "Lane follow active. mode=%s error=%.3f steering=%.3f strength=%.1f rows=%d",
                    self.follow_mode,
                    self.latest_error,
                    steering,
                    self.latest_strength,
                    self.rows_used,
                )
            rate.sleep()

        self.publish_cmd(0.0, 0.0)


def main():
    rospy.init_node("lane_follow_camera")
    LaneFollower().spin()


if __name__ == "__main__":
    main()
