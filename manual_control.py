#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image
from datetime import datetime
import os
from pynput import keyboard

class AckermannKeyboardControl(Node):
    def __init__(self):
        # Initialize the ROS node
        super().__init__('ackermann_keyboard_teleop')
        
        # Parameters
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_steering_angle', 1.0)
        self.declare_parameter('speed_increment', 0.5)
        self.declare_parameter('steering_increment', 0.2)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.steering_increment = self.get_parameter('steering_increment').value
        
        # Get the directory where the script is located
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.image_save_dir = os.path.join(self.script_dir, 'raw_images')
        
        # Create directory for saving images if it doesn't exist
        if not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)
            self.get_logger().info(f"Created directory for saving images: {self.image_save_dir}")
        
        # Initialize current speed and steering angle
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        
        # Track which keys are currently pressed
        self.keys_pressed = set()
        
        # CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Create publisher for Ackermann drive messages
        self.drive_pub = self.create_publisher(AckermannDrive, 'ackermann_cmd', 10)
        
        # Subscribe to camera topic
        self.camera_sub = self.create_subscription(
            Image, 
            '/gem/front_single_camera/front_single_camera/image_raw', 
            self.camera_callback, 
            10)
        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        
        # Create timer for control updates
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        
        # Display instructions
        self.print_instructions()
    
    def on_press(self, key):
        """Callback for key press events"""
        try:
            if key.char == 'w':
                self.keys_pressed.add('w')
            elif key.char == 's':
                self.keys_pressed.add('s')
            elif key.char == 'a':
                self.keys_pressed.add('a')
            elif key.char == 'd':
                self.keys_pressed.add('d')
            elif key.char == 'c':
                self.save_image()
            elif key.char == 'q':
                self.listener.stop()
                rclpy.shutdown()
        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.up:
                self.keys_pressed.add('w')
            elif key == keyboard.Key.down:
                self.keys_pressed.add('s')
            elif key == keyboard.Key.left:
                self.keys_pressed.add('a')
            elif key == keyboard.Key.right:
                self.keys_pressed.add('d')
            elif key == keyboard.Key.space:
                self.keys_pressed.clear()
                self.current_speed = 0.0
                self.current_steering_angle = 0.0
    
    def on_release(self, key):
        """Callback for key release events"""
        try:
            if key.char == 'w':
                self.keys_pressed.discard('w')
            elif key.char == 's':
                self.keys_pressed.discard('s')
            elif key.char == 'a':
                self.keys_pressed.discard('a')
            elif key.char == 'd':
                self.keys_pressed.discard('d')
        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.up:
                self.keys_pressed.discard('w')
            elif key == keyboard.Key.down:
                self.keys_pressed.discard('s')
            elif key == keyboard.Key.left:
                self.keys_pressed.discard('a')
            elif key == keyboard.Key.right:
                self.keys_pressed.discard('d')
    
    def camera_callback(self, msg):
        """Store the latest image from the camera"""
        self.latest_image = msg
    
    def print_instructions(self):
        """Print control instructions"""
        instructions = """
Ackermann Drive Keyboard Teleop Control
----------------------------------------
Control Keys:
  w/↑ - Move forward (hold)
  s/↓ - Move backward (hold)
  a/← - Steer left (hold)
  d/→ - Steer right (hold)
  space - Emergency stop
  q - Quit
  c - Capture and save current camera image
  
Current Settings:
  Max Speed: {0} m/s
  Max Steering Angle: {1} rad
  Speed Increment: {2} m/s
  Steering Increment: {3} rad
        """.format(self.max_speed, self.max_steering_angle, self.speed_increment, self.steering_increment)
        print(instructions)
    
    def publish_drive_msg(self):
        """Publish the current speed and steering angle as an AckermannDrive message"""
        msg = AckermannDrive()
        
        # Set drive parameters with instant response
        msg.speed = self.current_speed
        msg.steering_angle = self.current_steering_angle
        msg.steering_angle_velocity = 0.0  # No steering velocity limit
        msg.acceleration = 0.0  # No acceleration limit
        
        # Publish message
        self.drive_pub.publish(msg)
    
    def save_image(self):
        """Save the current camera image"""
        if self.latest_image is None:
            self.get_logger().warn("No camera image available to save")
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            # Create filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.image_save_dir, f"ackermann_image_{timestamp}.jpg")
            
            # Save image
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Image saved to: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")
    
    def update_controls(self):
        """Update speed and steering based on currently pressed keys"""
        # Reset speed and steering
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        
        # Update speed based on w/s keys
        if 'w' in self.keys_pressed:
            self.current_speed = self.max_speed
        elif 's' in self.keys_pressed:
            self.current_speed = -self.max_speed
            
        # Update steering based on a/d keys
        if 'a' in self.keys_pressed:
            self.current_steering_angle = self.max_steering_angle
        elif 'd' in self.keys_pressed:
            self.current_steering_angle = -self.max_steering_angle
    
    def timer_callback(self):
        """Callback for the timer that updates and publishes controls"""
        self.update_controls()
        self.publish_drive_msg()
    
    def shutdown(self):
        """Clean shutdown function"""
        # Stop vehicle before exiting
        self.keys_pressed.clear()
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        self.publish_drive_msg()
        self.get_logger().info("Stopping vehicle and exiting...")
        self.listener.stop()

def main(args=None):
    rclpy.init(args=args)
    
    controller = AckermannKeyboardControl()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
