#!/usr/bin/env python3 
from teleop_action_client import TeleopActionClient

# ROS Client Library for Python
import rclpy

# Initialize the rclpy library
rclpy.init()

# Create the Action Client node
action_client = TeleopActionClient()

# Send an example goal (example is default values of parameters) 
# Default values of send_goal: 
# operation="move-forward", duration=2.5
action_client.send_goal()
# Spin to execute callbacks
rclpy.spin(action_client)