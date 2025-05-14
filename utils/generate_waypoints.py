#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDrive
from septentrio_gnss_driver.msg import INSNavGeod
import os
from pynput import keyboard
import csv

class AckermannKeyboardControl:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ackermann_keyboard_teleop', anonymous=True)
        
        # Parameters
        self.max_speed = rospy.get_param('~max_speed', 5.0)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 1.0)
        self.speed_increment = rospy.get_param('~speed_increment', 0.5)
        self.steering_increment = rospy.get_param('~steering_increment', 0.2)
        
        # Get the directory where the script is located
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.waypoints_save_dir = os.path.join(self.script_dir, 'waypoints')
        
        # Create directory for saving waypoints if it doesn't exist
        if not os.path.exists(self.waypoints_save_dir):
            os.makedirs(self.waypoints_save_dir)
            rospy.loginfo(f"Created directory for saving waypoints: {self.waypoints_save_dir}")
        
        # Initialize current speed and steering angle
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        
        # Track which keys are currently pressed
        self.keys_pressed = set()
        
        # Variables for storing GPS data
        self.latest_gps = None
        self.start_latitude = None
        self.start_longitude = None
        self.waypoints = []
        
        # Create publisher for Ackermann drive messages
        self.drive_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10)
        
        # Subscribe to GNSS topic
        rospy.Subscriber('/septentrio_gnss/insnavgeod', INSNavGeod, self.gnss_callback)
        
        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        
        # Create timer for control updates
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)  # 50 Hz
        
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
                self.save_waypoint()
            elif key.char == 'q':
                self.listener.stop()
                rospy.signal_shutdown("User requested shutdown")
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
    
    def gnss_callback(self, msg):
        """Store the latest GNSS data and set starting position if not set yet"""
        self.latest_gps = msg
        
        # Store the starting position if not set yet
        if self.start_latitude is None and self.start_longitude is None:
            self.start_latitude = msg.latitude
            self.start_longitude = msg.longitude
            rospy.loginfo(f"Starting position set: Lat={self.start_latitude}, Long={self.start_longitude}")
    
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
  c - Capture and save current GPS waypoint
  
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
    
    def save_waypoint(self):
        """Save the current GPS coordinates relative to the starting position"""
        if self.latest_gps is None:
            rospy.logwarn("No GNSS data available to save")
            return
        
        try:
            # Calculate relative coordinates
            rel_latitude = self.latest_gps.latitude - self.start_latitude
            rel_longitude = self.latest_gps.longitude - self.start_longitude
            
            # Create a waypoint record
            waypoint = {
                'rel_latitude': rel_latitude,
                'rel_longitude': rel_longitude,
                'abs_latitude': self.latest_gps.latitude,
                'abs_longitude': self.latest_gps.longitude,
                'heading': self.latest_gps.heading,
            }
            
            # Add to waypoints list
            self.waypoints.append(waypoint)
            
            # Save to CSV file
            csv_filename = os.path.join(self.waypoints_save_dir, f"waypoints.csv")
            
            # Check if file exists to decide if we need to write headers
            file_exists = os.path.isfile(csv_filename)
            
            with open(csv_filename, 'a', newline='') as csvfile:
                fieldnames = ['rel_latitude', 'rel_longitude', 'abs_latitude', 'abs_longitude', 'heading']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                if not file_exists:
                    writer.writeheader()
                
                writer.writerow(waypoint)
            
            rospy.loginfo(f"Waypoint saved: Relative Lat={rel_latitude}, Relative Long={rel_longitude}")
            
        except Exception as e:
            rospy.logerr(f"Error saving waypoint: {e}")
    
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
    
    def timer_callback(self, event):
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
        rospy.loginfo("Stopping vehicle and exiting...")
        self.listener.stop()

def main():
    controller = AckermannKeyboardControl()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()

if __name__ == '__main__':
    main()
