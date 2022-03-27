# ezrassor_teleop

## What this is:  

This is currently a work-in-progress for the migration of the ezrassor_teleop_actions and ezrassor_teleop_msgs packages to ROS2.0.  
These packages come from the EZ-RASSOR GitHub repository: https://github.com/FlaSpaceInst/EZ-RASSOR  

The direct links to the packages are below:   
**ezrassor_teleop_actions:** https://github.com/FlaSpaceInst/EZ-RASSOR/tree/mainline/packages/actions/ezrassor_teleop_actions  
**ezrassor_teleop_msgs:** https://github.com/FlaSpaceInst/EZ-RASSOR/tree/mainline/packages/messages/ezrassor_teleop_msgs  

## How to compile it:

If you are wanting to test this yourself, here are the commands to build the packages from your ros2 workspace directory root:  

`colcon build --packages-select ezrassor_teleop_interfaces`  

`colcon build --packages-select ezrassor_teleop_actions`  

To verify whether the ezrassor_teleop_interfaces package was built, run this command:  

`ros2 interface show ezrassor_teleop_interfaces/action/Teleop`  

To rebuild the interfaces package after modifying it, make sure to run this command in order to do so:  

`colcon build --packages-select ezrassor_teleop_interfaces --cmake-clean-cache`  

## How to run/use it:

You'll need three terminals open, all of them sourced with your ROS2 Foxy installation and overlay  
In each terminal, navigate to ezrassor_teleop_actions/ezrassor_teleop_actions from the root of the repository folder

In the first terminal, run the teleop action server with the below command: 

`python3 teleop_action_server.py`  

In the second terminal run this command to display the messages published to the teleop_action_server/wheel_instructions_topic:

`ros2 topic echo /teleop_action_server/wheel_instructions_topic`  

In the third terminal, you can run one of three commands  
You can run `python3 send_goal_no_cancel.py` to send a goal without cancelling it  
You can run `python3 send_goal_timer_cancel.py` to send a goal that is cancelled halfway through execution  
You can run `python3 send_goal_keyboard_cancel.py` to send a goal that can be cancelled by pressing the ESC key  

## Current Progress:

Currently the teleop action server can recieve one goal at a time from the teleop action client  
The ability to send, execute, and cancel multiple goals is not yet complete
