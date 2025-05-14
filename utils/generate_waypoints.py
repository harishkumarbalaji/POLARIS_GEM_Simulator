#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
import os
from pynput import keyboard
import csv
import math
import tf.transformations

class AckermannKeyboardControl:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ackermann_keyboard_teleop', anonymous=True)
        
        # Parameters
        self.max_speed = rospy.get_param('~max_speed', 5.0)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 1.0)
        self.speed_increment = rospy.get_param('~speed_increment', 0.5)
        self.steering_increment = rospy.get_param('~steering_increment', 0.2)
        self.vehicle_name = rospy.get_param('~vehicle_name', 'gem_e4')
        self.recording_rate = rospy.get_param('~recording_rate', 0.3)  # Record every 0.3 seconds
        
        # Get the directory where the script is located
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.waypoints_save_dir = os.path.join(self.script_dir, 'waypoints')
        
        # Create directory for saving waypoints if it doesn't exist
        if not os.path.exists(self.waypoints_save_dir):
            os.makedirs(self.waypoints_save_dir)
            rospy.loginfo(f"Created directory for saving waypoints: {self.waypoints_save_dir}")
        
        # Initialize CSV file
        self.csv_filename = os.path.join(self.waypoints_save_dir, f"waypoints.csv")
        # Overwrite file if it exists (but don't write headers)
        with open(self.csv_filename, 'w') as f:
            pass
        rospy.loginfo(f"Created waypoints file: {self.csv_filename}")
        
        # Initialize current speed and steering angle
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        
        # Track which keys are currently pressed
        self.keys_pressed = set()
        
        # Variables for storing model states data
        self.model_states = None
        self.vehicle_index = None
        self.start_position = None
        self.start_orientation = None
        self.start_yaw = None  # Will be set when we get the first position
        self.waypoints = []
        self.is_recording = False
        
        # Create publisher for Ackermann drive messages
        self.drive_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10)
        
        # Subscribe to Gazebo model states topic
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        
        # Create timer for control updates (50 Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_timer_callback)
        
        # Create timer for waypoint recording (default: 0.3 Hz)
        self.recording_timer = None
        
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
            elif key.char == 'r':
                self.toggle_recording()
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
    
    def toggle_recording(self):
        """Toggle waypoint recording on/off"""
        if self.is_recording:
            # Stop recording
            self.is_recording = False
            if self.recording_timer:
                self.recording_timer.shutdown()
                self.recording_timer = None
            rospy.loginfo("Waypoint recording stopped")
        else:
            # Start recording if we have a starting position
            if self.start_position is None:
                rospy.logwarn("Cannot start recording - no starting position set yet")
                return
                
            self.is_recording = True
            self.recording_timer = rospy.Timer(rospy.Duration(self.recording_rate), self.recording_timer_callback)
            rospy.loginfo(f"Waypoint recording started (rate: {self.recording_rate}s)")
    
    def model_states_callback(self, msg):
        """Store the latest model states data and set starting position if not set yet"""
        self.model_states = msg
        
        # Find index of vehicle model
        if self.vehicle_name in msg.name:
            self.vehicle_index = msg.name.index(self.vehicle_name)
        else:
            # Try to find a matching model if exact name not found
            for i, name in enumerate(msg.name):
                if self.vehicle_name in name:
                    self.vehicle_index = i
                    self.vehicle_name = name
                    rospy.loginfo(f"Found vehicle model: {name}")
                    break
        
        # Store the starting position if not set yet
        if self.vehicle_index is not None and self.start_position is None:
            self.start_position = msg.pose[self.vehicle_index].position
            self.start_orientation = msg.pose[self.vehicle_index].orientation
            
            # Get the actual yaw from the starting orientation quaternion
            quaternion = (
                self.start_orientation.x,
                self.start_orientation.y,
                self.start_orientation.z,
                self.start_orientation.w
            )
            _, _, self.start_yaw = tf.transformations.euler_from_quaternion(quaternion)
            
            rospy.loginfo(f"Starting position set: x={self.start_position.x}, y={self.start_position.y}")
            rospy.loginfo(f"Starting yaw recorded: {math.degrees(self.start_yaw):.2f}°")
    
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
  r - Toggle waypoint recording (every {0} seconds)
  q - Quit
  
Current Settings:
  Max Speed: {1} m/s
  Max Steering Angle: {2} rad
  Speed Increment: {3} m/s
  Steering Increment: {4} rad
  Vehicle Model Name: {5}
        """.format(self.recording_rate, self.max_speed, self.max_steering_angle, 
                   self.speed_increment, self.steering_increment, self.vehicle_name)
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
        """Save the current position coordinates relative to the starting position"""
        if self.model_states is None or self.vehicle_index is None or self.start_position is None:
            rospy.logwarn("No vehicle position data available to save")
            return
        
        try:
            # Get current position and orientation
            current_position = self.model_states.pose[self.vehicle_index].position
            current_orientation = self.model_states.pose[self.vehicle_index].orientation
            
            # Calculate relative coordinates (x, y)
            rel_x = current_position.x - self.start_position.x
            rel_y = current_position.y - self.start_position.y
            
            # Calculate relative heading
            quaternion = (
                current_orientation.x,
                current_orientation.y,
                current_orientation.z,
                current_orientation.w
            )
            _, _, current_yaw = tf.transformations.euler_from_quaternion(quaternion)
            
            # Calculate heading relative to starting orientation
            relative_heading_rad = current_yaw - self.start_yaw
            
            # Convert to degrees and normalize to [-180, 180]
            relative_heading = math.degrees(relative_heading_rad)
            if relative_heading > 180:
                relative_heading -= 360
            elif relative_heading < -180:
                relative_heading += 360
                
            # Create a waypoint record - negate coordinates as requested
            waypoint = {
                'rel_x': -rel_x,
                'rel_y': -rel_y,
                'rel_heading': math.radians(relative_heading)
            }
            
            # Add to waypoints list
            self.waypoints.append(waypoint)
            
            # Write to CSV file (just the values, no headers)
            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([waypoint['rel_x'], waypoint['rel_y'], waypoint['rel_heading']])
            
            return waypoint
            
        except Exception as e:
            rospy.logerr(f"Error saving waypoint: {e}")
            return None
    
    def recording_timer_callback(self, event):
        """Callback for the timer that records waypoints at regular intervals"""
        if not self.is_recording:
            return
            
        waypoint = self.save_waypoint()
        if waypoint:
            rospy.loginfo(f"Recorded waypoint: x={waypoint['rel_x']:.3f}, y={waypoint['rel_y']:.3f}, heading={math.degrees(waypoint['rel_heading']):.2f}°")
    
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
    
    def control_timer_callback(self, event):
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
        
        # Stop recording if active
        if self.is_recording and self.recording_timer:
            self.recording_timer.shutdown()
        
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
