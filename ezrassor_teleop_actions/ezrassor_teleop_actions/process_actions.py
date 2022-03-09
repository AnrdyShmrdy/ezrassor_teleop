#!/usr/bin/env python
import sys
import rclpy
import teleop_action_client #changed from --> import ezrassor_teleop_actions

#changed client = ezrassor_teleop_actions.TeleopActionClient() to below line:
client = teleop_action_client.TeleopActionClient()

if len(sys.argv) <= 1:
    #rospy.logerr -->  client.teleop_node.get_logger().error(
    client.teleop_node.get_logger().error("Please provide a file to the client script...")
    exit(0)

instructions_file = sys.argv[1]


actions = client.read_instructions(instructions_file)
if not client.validate(actions):
    #rospy.logerr -->  client.teleop_node.get_logger().error(
    client.teleop_node.get_logger().error("Exiting client...")
    exit(0)

client.send_movement_goal(actions)
#rospy.loginfo --> client.teleop_node.get_logger().info(
client.teleop_node.get_logger().info("Actions completed. Closing client...")