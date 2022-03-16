#!/usr/bin/env python3 
from teleop_action_client import TeleopActionClient

# ROS Client Library for Python
import rclpy

# Initialize the rclpy library
rclpy.init()

# Create the Action Client node
action_client = TeleopActionClient()

# Send the goal  
action_client.send_goal_with_timer(operation="raise-front-arm",duration=1.0)
# Spin to execute callbacks
rclpy.spin(action_client)