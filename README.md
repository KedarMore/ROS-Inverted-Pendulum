# ROS-Inverted-Pendulum

Build your ROS Workspace and put the catkin package 'pendulum_control' in catkin_ws/src/

##

terminal 1:

"catkin_make"

"rosrun pendulum_control node_pendulum_controller"
##

terminal 2:

"rosrun rviz rviz"

Add > MarkerArray
##

terminal 3:

"rosrun pendulum_control node_pendulum_simulator"

This command will ask you to input an initial angle (in degrees).
