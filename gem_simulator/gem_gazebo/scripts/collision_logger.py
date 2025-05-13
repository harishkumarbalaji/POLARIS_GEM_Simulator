#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from datetime import datetime
import os

class CollisionLogger:
    def __init__(self):
        # Create logs directory if it doesn't exist
        self.logs_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs')
        os.makedirs(self.logs_dir, exist_ok=True)
        
        # Create log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.logs_dir, f'collision_log_{timestamp}.txt')
        
        # Initialize ROS node
        rospy.init_node('collision_logger', anonymous=True)
        
        # Subscribe to contact sensor topic
        rospy.Subscriber('/contact_sensor', ContactsState, self.contact_callback)
        
        rospy.loginfo(f"Collision logger initialized. Logging to: {self.log_file}")
        
    def contact_callback(self, msg):
        """Callback function for contact sensor messages"""
        if msg.states:  # If there are any contacts
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            for contact in msg.states:
                # Log collision information
                collision_info = (
                    f"Time: {timestamp}\n"
                    f"Collision between: {contact.collision1_name} and {contact.collision2_name}\n"
                    f"Contact position: x={contact.contact_positions[0].x:.3f}, "
                    f"y={contact.contact_positions[0].y:.3f}, "
                    f"z={contact.contact_positions[0].z:.3f}\n"
                    f"Contact normal: x={contact.contact_normals[0].x:.3f}, "
                    f"y={contact.contact_normals[0].y:.3f}, "
                    f"z={contact.contact_normals[0].z:.3f}\n"
                    f"Contact depth: {contact.depths[0]:.3f}\n"
                    f"{'='*50}\n"
                )
                
                # Write to log file
                with open(self.log_file, 'a') as f:
                    f.write(collision_info)
                
                rospy.loginfo(f"Collision detected and logged: {contact.collision1_name} - {contact.collision2_name}")

    def run(self):
        """Run the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = CollisionLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass 