#!/usr/bin/env python3 
from ezrassor_teleop_actions.teleop_action_client import TeleopActionClient
#library to use input arguments given to python file
import sys
# ROS Client Library for Python
import rclpy

# Initialize the rclpy library
rclpy.init()
action_client = TeleopActionClient()
if len(sys.argv) <= 1:
    action_client.get_logger().error("Please provide a file to the client script...")
    exit(0)

instructions_file = sys.argv[1]


actions = action_client.read_instructions(instructions_file)
if not action_client.validate(actions):
    action_client.get_logger().error("Exiting client...")
    exit(0)

action_client.send_goal_list_with_timer(actions)
action_client.get_logger().info("Actions completed. Closing client...")