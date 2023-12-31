## Robotic Arm Kinematics
6-DOF Robotic Arm Kinematics Implementation 


### Installation
Clone the repository using:

    git clone https://github.com/anubhav1772/robotic_arm_kinematics.git

Run catkin_make in your ROS source directory

    $ cd ~/catkin_ws
    $ catkin_make

Start the simulation using:

    $ roslaunch kuka_arm_gazebo arm_spawn.launch

Launch moveit and rviz:

    $ roslaunch kuka_arm_moveit_config simulation_execution.launch

### Requirements
* ROS Noetic (Ubuntu 20.04)
* Gazebo v11.11.0
