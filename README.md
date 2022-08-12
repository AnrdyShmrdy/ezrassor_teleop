# Ros2 ezrassor_teleop package

## What this is  

This is currently a work-in-progress for the migration of the ezrassor_teleop_actions and ezrassor_teleop_msgs packages to ROS2.0.  
These packages come from the EZ-RASSOR GitHub repository: <https://github.com/FlaSpaceInst/EZ-RASSOR>  

The direct links to the packages are below:
**ezrassor_teleop_actions:** <https://github.com/FlaSpaceInst/EZ-RASSOR/tree/mainline/packages/actions/ezrassor_teleop_actions>  
**ezrassor_teleop_msgs:** <https://github.com/FlaSpaceInst/EZ-RASSOR/tree/mainline/packages/messages/ezrassor_teleop_msgs>  

## How to compile it

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

## How to run/use it

You'll want to have at least two terminals to test this project. In each terminal, navigate to the folder that contains your ros2 workspace and run the command to source your ros2 environment:

`source install/setup.bash`

In the first terminal, run the teleop action server with the below command:

`ros2 run ezrassor_teleop_actions teleop_action_server`  

In the second terminal you can choose to run several different programs that showcase how the teleop_action_client can send goals  and interact with the teleop_action_server

To send a goal without cancelling it, run the send_goal_no_cancel program using the following command:
 
`ros2 run ezrassor_teleop_actions send_goal_no_cancel`

To send a goal that will be cancelled halfway through execution, run the send_goal_timer_cancel program using the following command:
 
`ros2 run ezrassor_teleop_actions send_goal_timer_cancel`  

(**Currently incomplete**) To send a goal that can be cancelled by pressing the ESC key, run the send_goal_keyboard_cancel using the following command:

`ros2 run ezrassor_teleop_actions send_goal_keyboard_cancel`  

To send a list of wheel operation goals from a text file, run the send_goal_list program and pass in the name of a text file with properly formatted commands. You can find an example of the format in the ezrassor_teleop/ezrassor_teleop_actions/ezrassor_teleop_actions/wheel-movements.txt. From the root of your ros2 workspace folder, you can run the program with the wheel-movements.txt file by running the following command (**NOTE: this is a single command, not multiple commands**):

`ros2 run ezrassor_teleop_actions send_goal_list src/ezrassor_teleop/ezrassor_teleop_actions/ezrassor_teleop_actions/wheel-movements.txt`

Finally, if you would like to see what is being published to the wheel_instructions_topic while running the above commands, feel free to open up another terminal, source your install/setup.bash file, and run this command to display the messages published to the /wheel_instructions_topic:

`ros2 topic echo /wheel_instructions_topic`  

## Summary of Current Status and Future Plans

* The non-gazebo related features in the original ROS1 ezrassor-teleop have been migrated to ROS2  
* Work still needs to be done before the gazebo related features can be migrated to ROS2
* Currently the ezrassor_teleop_actions package is being worked on actively
* The ezrassor_teleop_interfaces package is not being added to or worked on at the moment
* The send_goal_keyboard_cancel.py is being refactored to use a different method of keyboard input
* Further documentation of this package and the repository is being explored to make this easier to use

For a more up-to-date list of what is looking to be added, improved upon, or fixed in the package, see the list of issues for this repository.
