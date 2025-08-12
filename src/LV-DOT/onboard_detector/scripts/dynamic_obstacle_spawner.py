#!/usr/bin/env python3
"""
Dynamic Obstacle Movement Controller for TurtleBot3 Detector Simulation
This script controls the movement of dynamic obstacles in Gazebo
"""

import rospy
import math
import random
from geometry_msgs.msg import Twist

class DynamicObstacleSpawner:
    def __init__(self):
        rospy.init_node('dynamic_obstacle_spawner', anonymous=True)
        
        # Publishers for dynamic obstacles
        self.box_pub = rospy.Publisher('/dynamic_box_1/cmd_vel', Twist, queue_size=10)
        self.cylinder_pub = rospy.Publisher('/dynamic_cylinder/cmd_vel', Twist, queue_size=10)
        
        # Movement parameters
        self.spawn_rate = rospy.get_param('~spawn_rate', 0.1)  # 10 Hz
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s
        
        # Movement patterns
        self.box_time = 0.0
        self.cylinder_time = 0.0
        self.box_pattern = 0  # 0: circle, 1: linear, 2: random
        self.cylinder_pattern = 1
        
        # Timer for movement updates
        self.timer = rospy.Timer(rospy.Duration(self.spawn_rate), self.update_movements)
        
        rospy.loginfo("Dynamic Obstacle Spawner initialized")
        rospy.loginfo(f"Update rate: {1.0/self.spawn_rate} Hz")

    def update_movements(self, event):
        """Update movement commands for dynamic obstacles"""
        
        # Update time counters
        self.box_time += self.spawn_rate
        self.cylinder_time += self.spawn_rate
        
        # Move box with circular pattern
        box_cmd = self.get_box_movement()
        self.box_pub.publish(box_cmd)
        
        # Move cylinder with linear pattern
        cylinder_cmd = self.get_cylinder_movement()
        self.cylinder_pub.publish(cylinder_cmd)
        
        # Occasionally change movement patterns
        if self.box_time > 10.0:  # Change pattern every 10 seconds
            self.box_pattern = (self.box_pattern + 1) % 3
            self.box_time = 0.0
            rospy.loginfo(f"Box changed to pattern {self.box_pattern}")
            
        if self.cylinder_time > 8.0:  # Change pattern every 8 seconds
            self.cylinder_pattern = (self.cylinder_pattern + 1) % 3
            self.cylinder_time = 0.0
            rospy.loginfo(f"Cylinder changed to pattern {self.cylinder_pattern}")

    def get_box_movement(self):
        """Generate movement commands for the box"""
        cmd = Twist()
        
        if self.box_pattern == 0:  # Circular movement
            cmd.linear.x = 0.3
            cmd.angular.z = 0.5
            
        elif self.box_pattern == 1:  # Linear back and forth
            period = 6.0  # seconds for one cycle
            phase = (self.box_time % period) / period * 2 * math.pi
            cmd.linear.x = 0.4 * math.sin(phase)
            cmd.angular.z = 0.0
            
        else:  # Random movement
            cmd.linear.x = random.uniform(-0.3, 0.5)
            cmd.angular.z = random.uniform(-0.8, 0.8)
            
        return cmd

    def get_cylinder_movement(self):
        """Generate movement commands for the cylinder"""
        cmd = Twist()
        
        if self.cylinder_pattern == 0:  # Circular movement (opposite direction)
            cmd.linear.x = 0.25
            cmd.angular.z = -0.4
            
        elif self.cylinder_pattern == 1:  # Figure-8 pattern
            period = 8.0
            phase = (self.cylinder_time % period) / period * 2 * math.pi
            cmd.linear.x = 0.3 * math.cos(phase)
            cmd.angular.z = 0.6 * math.sin(2 * phase)
            
        else:  # Stop and go pattern
            cycle_time = self.cylinder_time % 4.0
            if cycle_time < 2.0:
                cmd.linear.x = 0.4
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.0
            
        return cmd

    def run(self):
        """Main execution loop"""
        rospy.loginfo("Starting dynamic obstacle movement patterns...")
        rospy.spin()

if __name__ == '__main__':
    try:
        spawner = DynamicObstacleSpawner()
        spawner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Dynamic obstacle spawner shutting down...")
        pass