# ezrassor_teleop

## What this is:  

This is currently a work-in-progress for the migration of the ezrassor_teleop_actions and ezrassor_teleop_msgs packages to ROS2.0.  
These packages come from the EZ-RASSOR GitHub repository: https://github.com/FlaSpaceInst/EZ-RASSOR  

The direct links to the packages are below:   
**ezrassor_teleop_actions:** https://github.com/FlaSpaceInst/EZ-RASSOR/tree/mainline/packages/actions/ezrassor_teleop_actions  
**ezrassor_teleop_msgs:** https://github.com/FlaSpaceInst/EZ-RASSOR/tree/mainline/packages/messages/ezrassor_teleop_msgs  

## How to compile it:

If you are wanting to test this yourself, clone this repository to the src folder in your ros2 workspace directory

Then navigate back to the root of your ros2 workspace directory and run either 

`colcon build --symlink-install`

or   

`colcon build --packages-select ezrassor_teleop_interfaces ezrassor_teleop_actions --symlink-install`  

Then after you have built the packages, run the command to source your ros2 environment:

`source install/setup.bash`

Before testing, verify whether the ezrassor_teleop_interfaces package was built, run this command:  

`ros2 interface show ezrassor_teleop_interfaces/action/Teleop`  

Note that if you modifiy the interfaces package. you will need run this command in order to rebuild it:  

`colcon build --packages-select ezrassor_teleop_interfaces --cmake-clean-cache`  

## How to run/use it:

You'll need three terminals open, all of them sourced with your ROS2 Foxy installation and overlay  
In each terminal, navigate to ezrassor_teleop_actions/ezrassor_teleop_actions from the root of the repository folder

In the first terminal, run the teleop action server with the below command: 

`python3 teleop_action_server.py`  

In the second terminal run this command to display the messages published to the teleop_action_server/wheel_instructions_topic:

`ros2 topic echo /teleop_action_server/wheel_instructions_topic`  

In the third terminal, you can run several different files of your choosing:    
You can run `python3 send_goal_no_cancel.py` to send a goal without cancelling it  
You can run `python3 send_goal_timer_cancel.py` to send a goal that is cancelled halfway through execution  
You can run `python3 send_goal_keyboard_cancel.py` to send a goal that can be cancelled by pressing the ESC key  
You can run `python3 send_goal_list.py wheel-movements.txt` to send a list of wheel operation goals from a text file
## Current Progress:

The non-gazebo related features in the original ROS1 ezrassor-teleop have been migrated to ROS2  
Detailed documentation regarding the package's functionality and testing is underway  

## Future plans:

See the list of issues for this repository to see what is looking to be added, improved upon, or fixed in the package
