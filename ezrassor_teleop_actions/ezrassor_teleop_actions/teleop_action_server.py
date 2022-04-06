#!/usr/bin/env python3
"""
NOTE:
In the past the action server depended on messages published by Gazebo 

While migrating, the parts that rely on Gazebo will be left out until 
the server can fully function without them. 

Then they will be added back, and the server can be configured to 
use them if desired.  
"""
#Import for goal queue implementation
import collections

#import python threading library
import threading

#import ROS2 multi-threading functions
from rclpy.executors import MultiThreadedExecutor

# Import our action definition
from ezrassor_teleop_interfaces.action import Teleop

# ROS Client Library for Python
import rclpy

# Action-related libraries for ROS 2 Python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

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
    #Bottom three lines for goal queue management
    self._goal_queue = collections.deque()
    self._goal_queue_lock = threading.Lock()
    self._current_goal = None
    #Added to determine if goal is in mid-execution
    self._executing_goal = False
    #TODO: Check if below three lines are actually necessary...
    self._goal_handle = None
    self._goal_lock = threading.Lock()
    self.teleop_goal = Teleop.Goal()
    # Instantiate a new action server
    # self, type of action, action name, callback function for executing goals
    self._action_server = ActionServer(
      self,
      Teleop,
      'Teleop',
      execute_callback=self.on_goal,
      goal_callback=self.goal_callback,
      handle_accepted_callback=self.handle_accepted_callback,
      cancel_callback=self.cancel_callback,
      callback_group=ReentrantCallbackGroup())
    
    #Create Publishers:
    self.wheel_instructions = self.create_publisher(
        Twist,
        "/wheel_instructions_topic",
        QUEUE_SIZE
    )
    self.front_arm_instructions = self.create_publisher(
        Float32,
        "/front_arm_instructions_topic",
        QUEUE_SIZE
    )
    self.back_arm_instructions = self.create_publisher(
        Float32,
        "/back_arm_instructions_topic",
        QUEUE_SIZE
    )
    self.front_drum_instructions = self.create_publisher(
        Float32,
        "/front_drum_instructions_topic",
        QUEUE_SIZE
    )
    self.back_drum_instructions = self.create_publisher(
        Float32,
        "/back_drum_instructions_topic",
        QUEUE_SIZE
    )

    # Make sure there are subscribers on the other end
    time.sleep(3)
    self.publish_stop()

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
  
  def destroy(self):
    self._action_server.destroy()
    super().destroy_node()

  def goal_callback(self, goal_request):
      """Accept or reject a client request to begin an action."""
      #print("*******goal_callback*******") #for testing purposes
      self.get_logger().info('Received goal request')
      return GoalResponse.ACCEPT

  def handle_accepted_callback(self, goal_handle):
    """Start or defer execution of an already accepted goal."""
    with self._goal_queue_lock:
        if self._current_goal is not None:
            # Put incoming goal in the queue
            self._goal_queue.append(goal_handle)
            self.get_logger().info('Goal put in the queue')
        else:
            # Start goal execution right away
            self._executing_goal = True
            self._current_goal = goal_handle
            self._current_goal.execute()

  def cancel_callback(self, goal):
      """Accept or reject a client request to cancel an action."""
      #print("*******cancel_callback*******") #for testing purposes
      self.get_logger().info('Received cancel request')
      return CancelResponse.ACCEPT
  def publish_stop(self):
    """Publish Zero to all topics to stop rover"""
    self.wheel_instructions.publish(ALL_STOP)
    self.front_arm_instructions.publish(float_zero)
    self.back_arm_instructions.publish(float_zero)
    self.front_drum_instructions.publish(float_zero)
    self.back_drum_instructions.publish(float_zero)
  def on_operation(self,operation):
    """Define what to do in response to certain operations"""
    #print("*******on_operation*******") #for testing purposes
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
      self.publish_stop()
  
  def on_goal(self, goal_handle):
    """Define all of the scenarios for handling a new goal from an action client."""
    #print("*******on_goal*******") #for testing purposes
    try:
      self.get_logger().info('Executing goal...')
      feedback = Teleop.Feedback()
      #Get operation to be executed
      operation = goal_handle.request.operation
      #Get length of time operation will be executed for
      duration = goal_handle.request.duration
      self.get_logger().info('Goal Operation: {}'.format(operation))
      self.get_logger().info('Goal Duration: {}'.format(duration))
      if self._executing_goal: #Sanity check to ensure messages are published correctly
        self.on_operation(operation) #Publish message based on operation recieved
      t0 = time.time()
      while (time.time() - t0) < duration and self._executing_goal:
        elapsed = time.time() - t0
        
        if goal_handle.is_cancel_requested:
          goal_handle.canceled()
          self._executing_goal = False
          self.get_logger().info('Goal canceled...publishing stop to all topics')
          self.publish_stop()
          self._logger.info("operation: {}, duration: {}, time spent executing: {}"
          .format(operation, duration, elapsed))
          return Teleop.Result()
        
        #Publish Feedback:
        print("operation: {}, duration: {} elapsed: {}"
        .format(operation, duration, elapsed), end='\r')
        feedback.x = 0.0
        feedback.y = 0.0
        feedback.heading = str("{} degrees".format(self.heading))
        try:
            goal_handle.publish_feedback(feedback)
        except:
            goal_handle.abort()
            return
          
      #indicate goal was successful
      goal_handle.succeed()
      self._executing_goal = False
      self.get_logger().info("Publishing stop to all topics")
      self.publish_stop() 
      self.get_logger().info('Goal Handle Successful!')
      print('------------------------------------------------------------------------------')
      # Create a result message of the action type
      result = Teleop.Result()
      
      #Update result
      result.x = feedback.x
      result.y = feedback.y
      
      return result
    finally:
      with self._goal_queue_lock:
          try:
              # Start execution of the next goal in the queue.
              self._current_goal = self._goal_queue.popleft()
              self.get_logger().info('Next goal pulled from the queue')
              self._executing_goal = True
              self._current_goal.execute()
          except IndexError:
              # No goal in the queue.
              self._current_goal = None
              self._executing_goal = False


  #Gazebo Dependent Functions:
  #TODO: Figure out way to remove gazebo dependency
  def gazebo_state_callback(self, data):
    """Saves the Gazebo link state (position) data for use in the server.
    Adapted from the ai_objects.py implementation"""
    #NOTE: Commented out below lines to prevent dependency errors
    # self.x = 0
    # self.y = 0
    # self.heading = ""

    # # Identify the index containing the position link state
    # index = 0
    # namespace = self.get_namespace()
    # namespace = namespace[1:-1] + "::base_link"

    # try:
    #     index = data.name.index(namespace)
    # except Exception:
    #     self.get_logger().debug("Failed to get index. Skipping...")
    #     return

    # # Extract the information
    # self.x = data.pose[index].position.x
    # self.y = data.pose[index].position.y
    # heading = self.quaternion_to_yaw(data.pose[index]) * 180 / math.pi

    # if heading > 0:
    #     self.heading = heading
    # else:
    #     self.heading = 360 + heading

  def quaternion_to_yaw(self, pose):
    """Helper function since Gazebo returns the forward kinematics of the robot
    as a quaternion"""
    #NOTE: Commented out below lines to prevent dependency errors
    # quaternion = (
    #     pose.orientation.x,
    #     pose.orientation.y,
    #     pose.orientation.z,
    #     pose.orientation.w,
    # )
    # #TODO: Refactor below lines once you understand the use of tf2 in euler transformations
    # euler = transformations.euler_from_quaternion(quaternion)
    # return euler[2]


def main(args=None):
  """
  Entry point for the program.
  """
  # Initialize the rclpy library
  rclpy.init(args=args)
  # Create the Action Server node
  teleop_action_server = TeleopActionServer()
  # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
  executor = MultiThreadedExecutor()
  #Spin the node to execute the callbacks
  rclpy.spin(teleop_action_server, executor=executor)
  # Shutdown the server
  teleop_action_server.destroy()
  #Shutdown
  rclpy.shutdown()

if __name__ == '__main__':
    main()
