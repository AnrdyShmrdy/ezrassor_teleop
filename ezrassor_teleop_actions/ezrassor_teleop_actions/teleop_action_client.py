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
from pynput import keyboard
#global variable responsible for tracking keyListenerStatus
_keyboardListenerActive = True
_secondsToCancel = 0.5
def on_press(key):
  global _keyboardListenerActive
  print('{0} pressed'.format(key))
  if key == keyboard.Key.esc:
    _keyboardListenerActive = False
    # Stop listener
    return False
_listener = keyboard.Listener(
on_press=on_press)
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
    """
    future is of type rclpy.task.Future
    future.result() is of type action_msgs.srv.CancelGoal_Response
    """
    cancel_response = future.result()
    if len(cancel_response.goals_canceling) > 0:
        self.get_logger().info('Goal successfully canceled')
    else:
        self.get_logger().info('Goal failed to cancel')

    rclpy.shutdown()

  def goal_response_callback(self, future): 
    """
    Get the goal_handle
    future is of type rclpy.task.Future
    future.result() is of type rclpy.action.client.ClientGoalHandle
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
      future is of type rclpy.task.Future
      future.result() is of type rclpy.action.client.ClientGoalHandle
      """
      global _secondsToCancel
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
      self._timer = self.create_timer(_secondsToCancel, self.timer_callback)

  def goal_response_callback_with_keyboard(self, future):
    """
    This function is used to test goal cancellation with keyboard
    future is of type rclpy.task.Future
    future.result() is of type rclpy.action.client.ClientGoalHandle
    """
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal rejected :(')
        return

    self._goal_handle = goal_handle

    self.get_logger().info('Goal accepted :)')
    _listener.start()
    self._timer = self.create_timer(0.05, self.keyboard_callback)
    #self._timer2 = self.create_timer(2.3, self.keyboard_callback2)
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback_with_keyboard)
    
  def keyboard_callback(self):
    """
    Function callback for keyboard goal cancellation
    Any function containing this callback will have "with_keyboard" appended to the name
    future is of type rclpy.task.Future
    self._goal_handle is of type ClientGoalHandle
    self._timer is of type rclpy.timer.Timer
    """
    global _keyboardListenerActive
    global _listener
    if(_keyboardListenerActive == False):
      _keyboardListenerActive = True
      self.get_logger().info('Cancelling goal...')
      # Below two lines create a goal cancellation request
      future = self._goal_handle.cancel_goal_async()
      future.add_done_callback(self.cancel_done)
      self.get_logger().info('Getting goal result...')
      # Below two lines create a goal result request
      future_result = self._goal_handle.get_result_async()
      future_result.add_done_callback(self.get_result_callback_with_keyboard)

  def feedback_callback(self, feedback_message):
    """
    feedback_message is of type ezrassor_teleop_interfaces.action.Teleop_FeedbackMessage
    feedback_message.feedback is of type ezrassor_teleop_interfaces.action.Teleop_Feedback
    """
    feedback = feedback_message.feedback
    self.get_logger().info("feedback: " + str(feedback))
    self.get_logger().info('heading: {0}'.format(feedback.heading))
    self.get_logger().info('feedback x: {0}'.format(feedback.x))
    self.get_logger().info('feedback y: {0}'.format(feedback.y))

  def timer_callback(self):
    """
    This function is used to test goal cancellation
    future is of type rclpy.task.Future
    self._goal_handle is of type ClientGoalHandle
    self._timer is of type rclpy.timer.Timer
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
    Gets the result after a non-keyboard goal is sent
    future is of type rclpy.task.Future
    future.result() is of type ezrassor_teleop_interfaces.action.Teleop_GetResult_Response
    future.result().result is of type ezrassor_teleop_interfaces.action.Teleop_Result
    """
    result = future.result().result
    self.get_logger().info("result: " + str(result))
    self.get_logger().info('result x: {0}'.format(result.x))
    self.get_logger().info('result y: {0}'.format(result.y))
    rclpy.shutdown()

  def get_result_callback_with_keyboard(self, future):
    """
    Gets the result after keyboard goal is sent
    future is of type rclpy.task.Future
    future.result() is of type ezrassor_teleop_interfaces.action.Teleop_GetResult_Response
    future.result().result is of type ezrassor_teleop_interfaces.action.Teleop_Result
    """
    global _listener
    #Stop Keyboard Listener Thread
    self.get_logger().info('Stopping Keyboard Listener...')
    _listener.stop()
    # Cancel the timer
    self.get_logger().info('Cancelling timer...')
    self._timer.cancel()
    #Get Result
    self.get_logger().info('Getting Result...')
    result = future.result().result
    self.get_logger().info("result: " + str(result))
    self.get_logger().info('result x: {0}'.format(result.x))
    self.get_logger().info('result y: {0}'.format(result.y))
    #Get Result
    self.get_logger().info('Shutting Down...')
    rclpy.shutdown()
  
  def send_goal(self, operation="move-forward", duration=2.5):
    """
    Action client to send the goal
    Default values represent an example/test goal to send
    goal_msg is of type ezrassor_teleop_interfaces.action.Teleop_Goal
    self._send_goal_future is of type rclpy.task.Future
    self._action_client is of type rclpy.action.client.ActionClient
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

  def send_goal_with_keyboard(self, operation="move-forward", duration=2.5):
    """
    Action client to send the goal and then cancel it with keyboard
    Default values represent an example/test goal to send
    goal_msg is of type ezrassor_teleop_interfaces.action.Teleop_Goal
    self._send_goal_future is of type rclpy.task.Future
    self._action_client is of type rclpy.action.client.ActionClient
    """
    # Set the goal message
    
    goal_msg = Teleop.Goal()
    goal_msg.operation = operation
    goal_msg.duration = duration
    self.get_logger().info("Goal to be sent: " + str(goal_msg))
    # Wait for the Action Server to launch
    self.get_logger().info('Waiting for Server...')
    self._action_client.wait_for_server()
    # Register a callback for when the future is complete
    self.get_logger().info('Registering callback for future complete...')
    self._send_goal_future = self._action_client.send_goal_async(
      goal_msg, 
      feedback_callback=self.feedback_callback)
    self.get_logger().info('Adding done callback...')    
    self._send_goal_future.add_done_callback(self.goal_response_callback_with_keyboard)

  def send_goal_with_timer(self, operation="move-forward", duration=2.5, secondsToCancel=1.25):
    """
    Action client to send the goal and then cancel it with timer
    Default values represent an example/test goal to send
    goal_msg is of type ezrassor_teleop_interfaces.action.Teleop_Goal
    self._send_goal_future is of type rclpy.task.Future
    self._action_client is of type rclpy.action.client.ActionClient
    """
    global _secondsToCancel
    _secondsToCancel = secondsToCancel 
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

  def send_goal_list(self, actions): #NOTE: STILL A WORK IN PROGRESS!!!!!
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

  def send_goal_list_with_timer(self, actions): #NOTE: STILL A WORK IN PROGRESS!!!!!
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