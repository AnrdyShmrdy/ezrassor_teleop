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
