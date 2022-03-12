#!/usr/bin/env python3

# Import our action definition
from ezrassor_teleop_interfaces.action import Teleop
# ROS Client Library for Python
import rclpy

# ActionServer library for ROS 2 Python
from rclpy.action import ActionServer

# Enables publishers, subscribers, and action servers to be in a single node
from rclpy.executors import MultiThreadedExecutor

# Handles the creation of nodes
from rclpy.node import Node

class TeleopActionServer(Node):
  """
  Create a ConnectToChargingDockActionServer class, 
  which is a subclass of the Node class.
  """
  def __init__(self):
  
    # Initialize the class using the constructor
    super().__init__('teleop_action_server')
    
    # Instantiate a new action server
    # self, type of action, action name, callback function for executing goals
    self._action_server = ActionServer(
      self,
      Teleop,
      'Teleop',
      execute_callback=self.on_goal)
      
  def on_goal(self, goal_handle):
    """
    Action server callback to execute accepted goals.
    """
    self.get_logger().info('Recieved goal, starting execution...')

    # Interim feedback
    feedback_msg = Teleop.Feedback()
    self.get_logger().info("feedback_msg: " + str(feedback_msg))
    self.get_logger().info("feedback x: " + str(feedback_msg.x))
    self.get_logger().info("feedback y: " + str(feedback_msg.y))

    self.get_logger().info("Goal Operation: " + str(goal_handle.request.operation))
    self.get_logger().info("Goal Duration: " + str(goal_handle.request.duration))

    # Update feedback  
    feedback_msg.heading = ''  
    feedback_msg.x = 0.0
    feedback_msg.y = 0.0
    goal_handle.publish_feedback(feedback_msg)
  
    # Indicate the goal was successful
    goal_handle.succeed()
    self.get_logger().info('CONNECTING...')
    self.get_logger().info('Successfully connected!')
  
    # Create a result message of the action type
    result = Teleop.Result()
    
    #Update result
    result.x = feedback_msg.x
    result.y = feedback_msg.y
    
    return result
    
def main(args=None):
  """
  Entry point for the program.
  """
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  try: 
  
    # Create the Action Server node
    teleop_action_server = TeleopActionServer()
    
    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(teleop_action_server)
    #executor.add_node(battery_state_subscriber)
    
    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
    finally:
      # Shutdown the nodes
      executor.shutdown()
      teleop_action_server.destroy_node()
      #battery_state_subscriber.destroy_node()

  finally:
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
