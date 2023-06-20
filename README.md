# ROS-Inverted-Pendulum

* /rosdistro: noetic
* /rosversion: 1.16.0

Build your ROS Workspace

"mkdir -p ~/catkin_ws/src"

"cd ~/catkin_ws/"

"catkin_make"

put the catkin package 'pendulum_control' in catkin_ws/src/

##

terminal 1:

"catkin_make"

"rosrun pendulum_control node_pendulum_controller"

Monitor Actual Angle here.

##

terminal 2:

"rosrun rviz rviz"

Add > MarkerArray
##

terminal 3:

"rosrun pendulum_control node_pendulum_simulator"

This command will ask you to input an initial angle (in degrees).

Monitor Motor Torque values here.
