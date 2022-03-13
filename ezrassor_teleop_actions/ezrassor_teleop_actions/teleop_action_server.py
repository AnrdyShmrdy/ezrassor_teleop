#!/usr/bin/env python3
"""
NOTE:
In the past the action server depended on messages published by Gazebo 

While migrating, the parts that rely on Gazebo will be left out until 
the server can fully function without them. 

Then they will be added back, and the server can be configured to use them if desired.  
"""
# Import our action definition
from ezrassor_teleop_interfaces.action import Teleop
# ROS Client Library for Python
import rclpy

# ActionServer library for ROS 2 Python
from rclpy.action import ActionServer

# Handles the creation of nodes
from rclpy.node import Node

import time
# Handle Twist messages, linear and angular velocity
from geometry_msgs.msg import Twist

from std_msgs.msg import Float32
float_zero = Float32()
float_neg_one = Float32()
float_pos_one = Float32()
float_zero.data = 0.0
float_neg_one.data = -1.0
float_pos_one.data = 1.0
ALL_STOP = Twist()
QUEUE_SIZE = 10
class TeleopActionServer(Node):

  def __init__(self):
  
    # Initialize the class using the constructor
    super().__init__('teleop_action_server')
    self.teleop_goal = Teleop.Goal()
    # Instantiate a new action server
    # self, type of action, action name, callback function for executing goals
    self._action_server = ActionServer(
      self,
      Teleop,
      'Teleop',
      execute_callback=self.on_goal)
    #Create Publishers:
    self.wheel_instructions = self.create_publisher(
        Twist,
        self.get_name() + "/wheel_instructions_topic",
        QUEUE_SIZE
    )
    self.front_arm_instructions = self.create_publisher(
        Float32,
        self.get_name() + "/front_arm_instructions_topic",
        QUEUE_SIZE
    )
    self.back_arm_instructions = self.create_publisher(
        Float32,
        self.get_name() + "/back_arm_instructions_topic",
        QUEUE_SIZE
    )
    self.front_drum_instructions = self.create_publisher(
        Float32,
        self.get_name() + "/front_drum_instructions_topic",
        QUEUE_SIZE
    )
    self.back_drum_instructions = self.create_publisher(
        Float32,
        self.get_name() + "/back_drum_instructions_topic",
        QUEUE_SIZE
    )

    # Make sure there are subscribers on the other end
    time.sleep(5)
    self.wheel_instructions.publish(ALL_STOP)
    self.front_arm_instructions.publish(float_zero)
    self.back_arm_instructions.publish(float_zero)
    self.front_drum_instructions.publish(float_zero)
    self.back_drum_instructions.publish(float_zero)

    # Subscribe to the link states so we can get the xy position from Gazebo
    #TODO: Come back to this after figuring out way to circumnavigate gazebo dependency
    # self.create_subscription(
    #     LinkStates, "/gazebo/link_states", self.gazebo_state_callback
    # )

    # Init the timer counter
    self._counter = 0

    # Initialize positional/state data.
    #TODO: Rename these three variables so people don't confuse them for the msg variables
    self.x = 0
    self.y = 0
    self.heading = ""

    # Give a success message!
    self.get_logger().info(
        "EZRASSOR Teleop Action Server has been initialized")
  #TODO: Come back to this after figuring out way to circumnavigate gazebo dependency
  # def gazebo_state_callback(self, data):
  #   """Saves the Gazebo link state (position) data for use in the server.
  #   Adapted from the ai_objects.py implementation"""

  #   self.x = 0
  #   self.y = 0
  #   self.heading = ""

  #   # Identify the index containing the position link state
  #   index = 0
  #   namespace = self.get_namespace()
  #   namespace = namespace[1:-1] + "::base_link"

  #   try:
  #       index = data.name.index(namespace)
  #   except Exception:
  #       self.get_logger().debug("Failed to get index. Skipping...")
  #       return

  #   # Extract the information
  #   self.x = data.pose[index].position.x
  #   self.y = data.pose[index].position.y
  #   heading = self.quaternion_to_yaw(data.pose[index]) * 180 / math.pi

  #   if heading > 0:
  #       self.heading = heading
  #   else:
  #       self.heading = 360 + heading

  # def quaternion_to_yaw(self, pose):
  #   """Helper function since Gazebo returns the forward kinematics of the robot
  #   as a quaternion"""
  #   quaternion = (
  #       pose.orientation.x,
  #       pose.orientation.y,
  #       pose.orientation.z,
  #       pose.orientation.w,
  #   )
  #   #TODO: Refactor below lines once you understand the use of tf2 in euler transformations
  #   euler = transformations.euler_from_quaternion(quaternion)
  #   return euler[2]
  def on_operation(self,operation):
    """Define what to do in response to certain operations"""
    msg = Twist()
    if operation == Teleop.Goal.MOVE_FORWARD_OPERATION:
      msg.linear.x = float(1)
      self.wheel_instructions.publish(msg)

    elif operation == Teleop.Goal.MOVE_BACKWARD_OPERATION:
      msg.linear.x = float(-1)
      self.wheel_instructions.publish(msg)

    elif operation == Teleop.Goal.ROTATE_LEFT_OPERATION:
      msg.angular.z = float(1)
      self.wheel_instructions.publish(msg)

    elif operation == Teleop.Goal.ROTATE_RIGHT_OPERATION:
      msg.angular.z = float(-1)
      self.wheel_instructions.publish(msg)

    elif operation == Teleop.Goal.RAISE_FRONT_ARM_OPERATION:
      self.front_arm_instructions.publish(float_pos_one)

    elif operation == Teleop.Goal.LOWER_FRONT_ARM_OPERATION:
      self.front_arm_instructions.publish(float_neg_one)

    elif operation == Teleop.Goal.RAISE_BACK_ARM_OPERATION:
      self.back_arm_instructions.publish(float_pos_one)

    elif operation == Teleop.Goal.LOWER_BACK_ARM_OPERATION:
      self.back_arm_instructions.publish(float_neg_one)

    elif operation == Teleop.Goal.DIG_FRONT_DRUM_OPERATION:
      self.front_drum_instructions.publish(float_pos_one)

    elif operation == Teleop.Goal.DUMP_FRONT_DRUM_OPERATION:
      self.front_drum_instructions.publish(float_neg_one)

    elif operation == Teleop.Goal.DIG_BACK_DRUM_OPERATION:
      self.back_drum_instructions.publish(float_pos_one)

    elif operation == Teleop.Goal.DUMP_BACK_DRUM_OPERATION:
      self.back_drum_instructions.publish(float_neg_one)

    else:
      # Otherwise, we must be stopping
      self.wheel_instructions.publish(ALL_STOP)
      self.front_arm_instructions.publish(float_zero)
      self.back_arm_instructions.publish(float_zero)
      self.front_drum_instructions.publish(float_zero)
      self.back_drum_instructions.publish(float_zero)
  def on_goal(self, goal_handle):
    """Define all of the scenarios for handling a new goal from an action client."""
    #Result Lines commented out for testing purposes
    #TODO: Figure out how to migrate these lines to ROS2
    # if not isinstance(goal_handle, Teleop):
    #     #result = Teleop.Result()
    #     self.get_logger().error("Unknown goal received")
    #     #TODO: Figure out what the below line was attempting to do.
    #     #TODO: Maybe replace with goal_handle.abort(result)?
    #     #self._server.set_aborted(result)  # Returns failure
    #     return
    self.executing_goal = True

    self.get_logger().info('Recieved goal, starting execution...')
    #Get operation to be executed
    operation = goal_handle.request.operation
    #Get length of time operation will be executed for
    duration = goal_handle.request.duration
    #Output name of operation
    self.get_logger().info("Goal Operation: " + str(operation))
    #Output duration of operation
    self.get_logger().info("Goal Duration: " + str(duration))
    #Determine what to publish to topics based on operation type:
    self.on_operation(operation)
    time.sleep(1) #wait added to test published output
    # Interim feedback
    feedback_msg = Teleop.Feedback()
    self.get_logger().info("feedback_msg: " + str(feedback_msg))
    self.get_logger().info("feedback x: " + str(feedback_msg.x))
    self.get_logger().info("feedback y: " + str(feedback_msg.y))

    # Update feedback  
    feedback_msg.heading = ''  
    feedback_msg.x = 0.0
    feedback_msg.y = 0.0
    goal_handle.publish_feedback(feedback_msg)
  
    # Indicate the goal was successful
    goal_handle.succeed()
    self.get_logger().info('Goal Handle Successful!')
  
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
    
    try:
      # Spin the node to execute the callbacks
      rclpy.spin(teleop_action_server)
    finally:
      # Shutdown the node
      teleop_action_server.destroy_node()

  finally:
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
