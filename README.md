# **DIFFERENTIAL ROBOT ON ROS**
> *Author: Paparella Santorsola Vito*

This package generates a differential robot with URDF. There are several  launch files in order to spawn the model in RVIZ, GAZEBO or both.  
The robot could be moved with the rqt steering gui or with the custom included python scripts. The robot is equipped with a visual sensor.

## **FILE INFO**
### **URDF FILES**
* diff_robot_din.urdf - geometry, collisions, inertia, transmissions, visual sensor and gazebo integration
* diff_robo_noInertia.urdf - geometry and collisions only
* cylinder_wheel.urdf - standard wheel geometry, collisions, inertia and transmission; included in diff_robot_din.urdf
* visual_sensor.xacro - visual sensor description; includend in diff_robot_din.urdf

### **LAUNCH FILES**
* robot_state.launch - loads the robot_descriptio parameter and starts robot_state_publisher; included in all launch files
* dr_rviz.launch - loads the robot in rviz and enables the joint_state_gui
* dr_gaz.launch - spawns the robot in gazebo and enable rqt_steering control
* dr_full.launch - all of the previous

### **PYTHON SCRIPTS**
* basic_move.py - test script; makes the robot move forward
* diff_robot_move.py - enable a simple keyboard control of the robot using curses library
