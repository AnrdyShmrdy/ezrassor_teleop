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

#variable for tracking keyListenerStatus
#_keyboardListenerActive = True #TODO: Need to refactor use of this variable. See https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22

#variable passed to createTimer in goal_response_callback_with_timer
_secondsToCancel = 0.5

#TODO: Need to refactor on_press. See https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22
#function responsible for determining if a key is pressed
# def on_press(key):
#   global _keyboardListenerActive
#   #print('{0} pressed'.format(key))
#   if key == keyboard.Key.esc:
#     #keyboardListener should stop
#     _keyboardListenerActive = False
#     # Stop listener by returning false
#     return False
# _listener = keyboard.Listener(
# on_press=on_press)

"""
Terminology:
Callbacks: Units of work executed by threads. 
           These include things like subscription callbacks, timer callbacks, 
           service calls, and received client responses.
Callback group: controls when callbacks are allowed to be executed. 
Task: callback or coroutine that returns a Future. 
Future: represents the outcome of a task
Executor: Tasks, callbacks, coroutines, etc. are added to these. 
          They control which threads the tasks/callbacks/coroutines 
          get executed in. They execute them whenever a call to the
          "spin" method of an executor is made.
Goal handle: used to monitor the status of the goal and get the final result. 
"""

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
    Determine whether or not the goal was successfully cancelled
    """
    #Helpful Debug info:
    #future is of type rclpy.task.Future
    #future.result() is of type action_msgs.srv.CancelGoal_Response
    
    #Get the result from attempting to cancel the task/goal
    cancel_response = future.result()
    #Check and see if the goal was cancelled
    if len(cancel_response.goals_canceling) > 0:
        self.get_logger().info('Goal successfully canceled')
    else:
        self.get_logger().info('Goal failed to cancel')

  def goal_response_callback(self, future): 
    """
    This function is called when no cancel method is specified in send_goal
    """
    #Useful Debug Info:
    #future is of type rclpy.task.Future
    #future.result() is of type rclpy.action.client.ClientGoalHandle

    #Get the goal handle 
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
      This function is called when cancel_method="timer" in send_goal
      """
      #Helpful Debug Info:
      #future is of type rclpy.task.Future
      #future.result() is of type rclpy.action.client.ClientGoalHandle

      global _secondsToCancel
      goal_handle = future.result()
      if not goal_handle.accepted:
          self.get_logger().info('Goal rejected :(')
          return

      self._goal_handle = goal_handle

      self.get_logger().info('Goal accepted :)')

      # Start the timer to execute callback in _secondsToCancel seconds
      self._timer = self.create_timer(_secondsToCancel, self.timer_callback)
  
  #TODO: Need to refactor goal_response_callback_with_keyboard. See https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22
  # def goal_response_callback_with_keyboard(self, future):
  #   """
  #   This function is used to test goal cancellation with keyboard
  #   """
  #   #Helpful Debug Info:
  #   #future is of type rclpy.task.Future
  #   #future.result() is of type rclpy.action.client.ClientGoalHandle
    
  #   goal_handle = future.result()
  #   if not goal_handle.accepted:
  #       self.get_logger().info('Goal rejected :(')
  #       return

  #   self._goal_handle = goal_handle

  #   self.get_logger().info('Goal accepted :)')
  #   _listener.start()
  #   self._timer = self.create_timer(timer_period_sec=0.05, 
  #                                   callback=self.keyboard_callback)
  #   self._get_result_future = goal_handle.get_result_async()
  #   self._get_result_future.add_done_callback(self.get_result_callback)

  ##TODO: Need to refactor keyboard_callback. See https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22
  # def keyboard_callback(self):
  #   """
  #   Function callback for keyboard goal cancellation
  #   This function checks if the keyboardListener should still be active
  #   If it should be terminated, then it stops the listener
  #   If not then it runs without doing anything
  #   """
  #   #Helpful Debug Info:
  #   #future is of type rclpy.task.Future
  #   #self._goal_handle is of type ClientGoalHandle
  #   #NOTE:Any function containing this callback will have "with_keyboard" appended to the name
  #   global _keyboardListenerActive
  #   global _listener
  #   #If key is pressed, then value of _keyboardListenerActive is False
  #   if(_keyboardListenerActive == False):
  #     #Set value back to True so that we can rerun the keyboard listener again later
  #     _keyboardListenerActive = True
  #     #Stop Keyboard Listener Thread
  #     self.get_logger().info('Stopping Keyboard Listener...')
  #     _listener.stop()
  #     # Cancel the timer
  #     self.get_logger().info('Cancelling timer...')
  #     self._timer.cancel()
  #     self.get_logger().info('Cancelling goal...')
  #     # Below two lines create a goal cancellation request
  #     future = self._goal_handle.cancel_goal_async()
  #     future.add_done_callback(self.cancel_done)

  def feedback_callback(self, feedback_message):
    """
    This function is called every time feedback is published
    It's purpose is to print the new feedback that gets published
    """
    #Helpful Debug Info:
    #feedback_message is of type ezrassor_teleop_interfaces.action.Teleop_FeedbackMessage
    #feedback_message.feedback is of type ezrassor_teleop_interfaces.action.Teleop_Feedback

    feedback = feedback_message.feedback
    self.get_logger().info("feedback: " + str(feedback))
    self.get_logger().info('heading: {0}'.format(feedback.heading))
    self.get_logger().info('feedback x: {0}'.format(feedback.x))
    self.get_logger().info('feedback y: {0}'.format(feedback.y))

  def timer_callback(self):
    """
    This function is called by the timer created in goal_response_callback_with_timer
    after timer_period_sec number of seconds has elapsed
    """
    #Helpful Debug Info:
    #future is of type rclpy.task.Future
    #self._goal_handle is of type ClientGoalHandle
    #self._action_client is of type rclpy.action.client.ActionClient
    #self._timer is of type rclpy.timer.Timer
    #NOTE: Any function containing this callback will have "with_timer" appended to the name
    self.get_logger().info('Canceling goal')
    # Below two lines create a goal cancellation request
    future = self._goal_handle.cancel_goal_async()
    future.add_done_callback(self.cancel_done)

    # Cancel the timer
    self._timer.cancel()

  def get_result_callback(self, future):
    """
    Gets the result of a goal
    """
    #Helpful Debug Info:
    #future is of type rclpy.task.Future
    #future.result() is of type ezrassor_teleop_interfaces.action.Teleop_GetResult_Response
    #self._action_client is of type rclpy.action.client.ActionClient
    #future.result().result is of type ezrassor_teleop_interfaces.action.Teleop_Result
    try:
      global _listener
      #Stop Keyboard Listener Thread
      self.get_logger().info('Stopping Keyboard Listener...')
      _listener.stop()
      # Cancel the timer
      self.get_logger().info('Cancelling timer...')
      self._timer.cancel()
    except: #Added in case error is raised upon cancelling a previously cancelled timer
      self.get_logger().info('Skipped cancelling timer and/or keyboard listener') #TODO: Maybe reword this after dealing with https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22
    finally:
      result = future.result().result
      self.get_logger().info("result: " + str(result))
      self.get_logger().info('result x: {0}'.format(result.x))
      self.get_logger().info('result y: {0}'.format(result.y))
      rclpy.shutdown()
  
  def send_goal(self, 
                cancel_method="", 
                operation="move-forward", 
                duration=2.5, 
                secondsToCancel=1.25):
    """
    Action client to send the goal
    Methods for cancelling: keyboard and timer 
    Default values represent an example/test goal to send
    """
    #TODO: Maybe reword the above doc comment after dealing with https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22

    #Helpful Debug Info:
    #goal_msg is of type ezrassor_teleop_interfaces.action.Teleop_Goal
    #self._send_goal_future is of type rclpy.task.Future
    #self._action_client is of type rclpy.action.client.ActionClient

    global _secondsToCancel
    _secondsToCancel = secondsToCancel 

    # Initialize the goal message
    goal_msg = Teleop.Goal()
    # Set the type of operation to execute
    goal_msg.operation = operation
    # Set the duration to execute the operation
    goal_msg.duration = duration
    # Log the string value of the goal message for debug purposes
    self.get_logger().info("Goal to be sent: " + str(goal_msg))
    # Wait for the Action Server to become available
    self._action_client.wait_for_server()
    # Send a goal, asynchronously get the result, and register
    # a callback function to give feedback associated with the goal
    self._send_goal_future = self._action_client.send_goal_async(
      goal=goal_msg, 
      feedback_callback=self.feedback_callback)
    
    #TODO: Need to refactor this if-statement. See https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22
    # #If method of cancelling a goal is keyboard:
    # if(cancel_method == "keyboard"):
    #   #add a callback function to be executed when the Future is complete
    #   self._send_goal_future.add_done_callback(self.goal_response_callback_with_keyboard)
    
    #If method of cancelling a goal is timer:
    if(cancel_method == "timer"):
      #add a callback function to be executed when the Future is complete
      self._send_goal_future.add_done_callback(self.goal_response_callback_with_timer)
    #If method of cancellation is not given:
    else:
      #add a callback function to be executed when the Future is complete
      self._send_goal_future.add_done_callback(self.goal_response_callback)

  def send_goal_list(self, actions): #NOTE: STILL A WORK IN PROGRESS!!!!!
    ##NOTE: THIS DOES NOT YET FUNCTION CORRECTLY.
    #TODO: Prevent this from sending a goal until previous one is either finished, aborted, rejected, or cancelled
    """Accepts a list of actions to send to the action server.
    Feedback returned indicates the robot's current x, y, and heading."""
    for action in actions:
        instruction = action.split(" ")
        self.send_goal("",instruction[0], float(instruction[1]))
        #Need to sleep so that goals are sent in order recieved (queue)
        #Without this, goals tend to be executed in reverse order (stack)
        time.sleep(0.01)

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
            self.get_logger().error("Invalid operation received: {0}".format(action))
            return False

        #TODO: Python2 did not support isnumeric, so this was to check for cast exceptions. Refactor to use Python3 isnumeric
        try:
            float(duration)
        except ValueError:
            self.get_logger().error(
                "Non-number passed in for time duration: \n{0}".format(action)
            )

    return True