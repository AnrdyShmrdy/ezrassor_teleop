
#!/usr/bin/env python3 
from teleop_action_client import TeleopActionClient

# ROS Client Library for Python
import rclpy

def main(args=None):
  """
  Entry point for the program
  """
  # Initialize the rclpy library
  rclpy.init()

  # Create the Action Client node
  action_client = TeleopActionClient()

# Send an example goal (example is default values of parameters) 
# Default values of send_goal: 
# cancel_method="", operation="move-forward", duration=2.5, secondsToCancel=1.25

#  action_client.send_goal(cancel_method="keyboard") #TODO: Uncomment after resolving https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22
  # Spin to execute callbacks
#  rclpy.spin(action_client) #TODO: Uncomment after resolving https://github.com/AnrdyShmrdy/ezrassor_teleop/issues/22

if __name__ == '__main__':
    main()