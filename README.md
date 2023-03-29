# vcu_am_post_processing
Turn robot on and enable
Load ros_init_grip program
Enable remote control mode

Initializes node for communication
ros2 launch ur_robot_driver ur5e.launch.py robot_ip:=192.168.1.102 launch_rviz:=true

Plays program to enable ROS control of UR
ros2 service call /dashboard_client/play std_srvs/srv/Trigger

Starts moving robot based on movements listed 
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py

Read position of tool0 to base link
ros2 run tf2_ros tf2_echo tool0 base
