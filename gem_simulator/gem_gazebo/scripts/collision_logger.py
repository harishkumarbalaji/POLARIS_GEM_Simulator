#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from datetime import datetime
import os

class CollisionLogger:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('collision_logger', anonymous=True)
        
        # Create logs directory if it doesn't exist
        self.logs_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'logs')
        os.makedirs(self.logs_dir, exist_ok=True)
        
        # Create log file with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.logs_dir, f'collision_log_{timestamp}.txt')
        
        # Subscribe to the contact sensor topic
        rospy.Subscriber('/contact_sensor', ContactsState, self.contact_callback)
        
        rospy.loginfo("[Info] Collision logger initialized. Logging to: %s", self.log_file)
        rospy.loginfo("[Info] Waiting for collision events...")

    def simplify_collision_name(self, name):
        """Simplify collision names to be more readable"""
        if "base_link" in name:
            return "vehicle body"
        elif "front_rack" in name:
            return "vehicle front bumper"
        elif "rear_rack" in name:
            return "vehicle rear bumper"
        elif "::" in name:
            # For other objects, just return the first part
            return name.split("::")[0]
        return name

    def contact_callback(self, msg):
        if not msg.states:  # If no contacts
            return
            
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # Create a dictionary to store unique collisions
        unique_collisions = {}
        
        # Process all collision states
        for state in msg.states:
            # Create a unique key for this collision pair
            collision_key = tuple(sorted([state.collision1_name, state.collision2_name]))
            
            # If this collision pair hasn't been seen before, or if this collision has a greater depth
            if collision_key not in unique_collisions or state.depths[0] > unique_collisions[collision_key].depths[0]:
                unique_collisions[collision_key] = state
        
        # Print to terminal
        rospy.loginfo("\n=== Collision Detected at %s ===", timestamp)
        
        with open(self.log_file, 'a') as f:
            f.write(f"\n=== Collision Detected at {timestamp} ===\n")
            
            for i, (collision_key, state) in enumerate(unique_collisions.items(), 1):
                # Simplify collision names
                collision1 = self.simplify_collision_name(state.collision1_name)
                collision2 = self.simplify_collision_name(state.collision2_name)
                
                # Reorder collisions to put vehicle parts first
                if "vehicle" in collision2:
                    collision1, collision2 = collision2, collision1
                
                # Print to terminal
                rospy.loginfo("\nContact %d:", i)
                rospy.loginfo("Collision between: %s and %s", collision1, collision2)
                
                # Combine position coordinates into one line
                pos = state.contact_positions[0]
                pos_str = f"Contact Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}"
                rospy.loginfo(pos_str)
                
                # Combine normal coordinates into one line
                normal = state.contact_normals[0]
                normal_str = f"Contact Normal: x={normal.x:.3f}, y={normal.y:.3f}, z={normal.z:.3f}"
                rospy.loginfo(normal_str)
                
                depth_str = f"Contact Depth: {state.depths[0]:.3f}"
                rospy.loginfo(depth_str)
                
                # Write to file
                f.write(f"\nContact {i}:\n")
                f.write(f"Collision between: {collision1} and {collision2}\n")
                f.write(f"{pos_str}\n")
                f.write(f"{normal_str}\n")
                f.write(f"{depth_str}\n")
                f.write("-" * 50 + "\n")
            
            f.write("\n")
            rospy.loginfo("-" * 50)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = CollisionLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass 