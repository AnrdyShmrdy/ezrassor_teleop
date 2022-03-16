#!/usr/bin/env python3 

# Import our action definition
from ezrassor_teleop_interfaces.action import Teleop
# ROS Client Library for Python
import rclpy
import time
# ActionClient library for ROS 2 Python
from rclpy.action import ActionClient

# Handles the creation of nodes
from rclpy.node import Node

class TeleopActionClient(Node):

  def __init__(self):
  
    # Initialize the class using the constructor
    super().__init__('teleop_action_client')
    
    # Instantiate a new action client
    # self, type of action, action name
    self._action_client = ActionClient(
      self,
      Teleop,
      'Teleop')

  def cancel_done(self, future):
      cancel_response = future.result()
      if len(cancel_response.goals_canceling) > 0:
          self.get_logger().info('Goal successfully canceled')
      else:
          self.get_logger().info('Goal failed to cancel')

      rclpy.shutdown()

  def goal_response_callback(self, future): 
    """
    Get the goal_handle
    """
    goal_handle = future.result()

    if not goal_handle.accepted:
      self.get_logger().info('Goal rejected...HOUSTON, WE HAVE A PROBLEM!')
      return
    
    self._goal_handle = goal_handle

    self.get_logger().info('Goal accepted...')
    
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback)

  def goal_response_callback_with_timer(self, future):
      """
      This function is used to test goal cancellation
      """
      goal_handle = future.result()
      if not goal_handle.accepted:
          self.get_logger().info('Goal rejected :(')
          return

      self._goal_handle = goal_handle

      self.get_logger().info('Goal accepted :)')

      # Start a 0.5 second timer. 
      # The sample goal sent to the server is 1 second in duration
      # We set the timer for 0.5 seconds so that it runs the callback
      # function and cancels the goal mid-execution
      self._timer = self.create_timer(0.5, self.timer_callback)


  def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info("feedback: " + str(feedback))
    self.get_logger().info('heading: {0}'.format(feedback.heading))
    self.get_logger().info('feedback x: {0}'.format(feedback.x))
    self.get_logger().info('feedback y: {0}'.format(feedback.y))

  def timer_callback(self):
    """
    This function is used to test goal cancellation
    """
    #NOTE: Any function containing this callback will have "with_timer" appended to the name
    self.get_logger().info('Canceling goal')
    # Cancel the goal
    future = self._goal_handle.cancel_goal_async()
    future.add_done_callback(self.cancel_done)

    # Cancel the timer
    self._timer.cancel()

  def get_result_callback(self, future):
    """
    Gets the result 
    """
    result = future.result().result
    self.get_logger().info("result: " + str(result))
    self.get_logger().info('result x: {0}'.format(result.x))
    self.get_logger().info('result y: {0}'.format(result.y))
    rclpy.shutdown()

  def send_goal(self, operation, duration):
    """
    Action client to send the goal
    """
    # Set the goal message
    goal_msg = Teleop.Goal()
    goal_msg.operation = operation
    goal_msg.duration = duration
    self.get_logger().info("Goal to be sent: " + str(goal_msg))
    # Wait for the Action Server to launch
    self._action_client.wait_for_server()
    # Register a callback for when the future is complete
    self._send_goal_future = self._action_client.send_goal_async(
      goal_msg, 
      feedback_callback=self.feedback_callback)    
    self._send_goal_future.add_done_callback(self.goal_response_callback)

  def send_goal_with_timer(self, operation, duration):
    """
    This function is used to test goal cancellation
    """
    # Set the goal message
    goal_msg = Teleop.Goal()
    goal_msg.operation = operation
    goal_msg.duration = duration
    self.get_logger().info("Goal to be sent: " + str(goal_msg))
    # Wait for the Action Server to launch
    self._action_client.wait_for_server()
    # Register a callback for when the future is complete
    self._send_goal_future = self._action_client.send_goal_async(
      goal_msg, 
      feedback_callback=self.feedback_callback)    
    self._send_goal_future.add_done_callback(self.goal_response_callback_with_timer)


  def send_goal_list(self, actions): 
    ##NOTE: THIS DOES NOT YET FUNCTION CORRECTLY.
    #TODO: Prevent this from sending a goal until previous one is either finished, aborted, rejected, or cancelled
    """Accepts a list of actions to send to the action server.
    Feedback returned indicates the robot's current x, y, and heading."""
    for action in actions:
        instruction = action.split(" ")
        operation = instruction[0]
        duration = float(instruction[1])
        goal_msg = Teleop.Goal()
        goal_msg.operation = operation
        goal_msg.duration = duration
        # Wait for the Action Server to launch
        self._action_client.wait_for_server()
        # Register a callback for when the future is complete
        self._send_goal_future = self._action_client.send_goal_async(
          goal_msg, 
          feedback_callback=self.feedback_callback)    
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info("{0} has been sent.".format(operation))
        #Wait to to account for lag and prevent multiple goals from being sent simultaneously
        time.sleep(0.1)

  def send_goal_list_with_timer(self, actions): ##STILL A WORK IN PROGRESS!!!!!
    ##NOTE: THIS DOES NOT YET FUNCTION CORRECTLY.
    #TODO: Prevent this from sending a goal until previous one is either finished, aborted, rejected, or cancelled
    """Accepts a list of actions to send to the action server.
    Feedback returned indicates the robot's current x, y, and heading."""
    for action in actions:
        instruction = action.split(" ")
        operation = instruction[0]
        duration = float(instruction[1])
        goal_msg = Teleop.Goal()
        goal_msg.operation = operation
        goal_msg.duration = duration
        # Wait for the Action Server to launch
        self._action_client.wait_for_server()
        # Register a callback for when the future is complete
        self._send_goal_future = self._action_client.send_goal_async(
          goal_msg, 
          feedback_callback=self.feedback_callback)    
        self._send_goal_future.add_done_callback(self.goal_response_callback_with_timer)

        self.get_logger().info("{0} has been sent.".format(operation))
        #Wait to to account for lag and prevent multiple goals from being sent simultaneously
        time.sleep(0.1)

  #Miscellaneous fuctions:

  def read_instructions(self, instructions_file):
    """Parse in a text file to identify new goals.
    Lines that start with # are considered comments and are ignored."""
    actions = []
    with open(instructions_file, "r") as reader:
        for line in reader.readlines():
            if line.startswith("#"):
                continue
            line = line.rstrip()
            actions.append(line.lower())

    return actions

  def validate(self, action_list):
    valid_goal = Teleop.Goal()
    """Check the list of parsed actions to ensure that they are all valid actions.
    If any single action is invalid, then the function will return False and the
    program will exit early."""

    if not type(action_list) is list:
        #rospy.logerr -->  self.teleop_node.get_logger().error(
        self.get_logger().error(
            "Teleop Client script was not given an object of type list"
        )
        return False

    valid_operations = [
        valid_goal.MOVE_FORWARD_OPERATION,
        valid_goal.MOVE_BACKWARD_OPERATION,
        valid_goal.ROTATE_LEFT_OPERATION,
        valid_goal.ROTATE_RIGHT_OPERATION,
        valid_goal.RAISE_FRONT_ARM_OPERATION,
        valid_goal.LOWER_FRONT_ARM_OPERATION,
        valid_goal.RAISE_BACK_ARM_OPERATION,
        valid_goal.LOWER_BACK_ARM_OPERATION,
        valid_goal.DUMP_FRONT_DRUM_OPERATION,
        valid_goal.DIG_FRONT_DRUM_OPERATION,
        valid_goal.DUMP_BACK_DRUM_OPERATION,
        valid_goal.DIG_BACK_DRUM_OPERATION,
        valid_goal.STOP_OPERATION,
    ]

    # Validate every single action
    for action in action_list:

        action_components = action.split(" ")

        if len(action_components) != 2:
            self.get_logger().error(
                "Incorrect number of instructions given. Expected 2 got {0}\n{1}"
                .format(
                len(action_components),
                action)
            )
            return False

        op = action_components[0]
        duration = action_components[1]

        if op not in valid_operations:
            #rospy.logerr -->  self.teleop_node.get_logger().error(
            self.get_logger().error("Invalid operation received: {0}".format(action))
            return False

        # Since Python2 does not support isnumeric, check for cast exceptions
        try:
            float(duration)
        except ValueError:
            self.get_logger().error(
                "Non-number passed in for time duration: \n{0}".format(action)
            )

    return True
