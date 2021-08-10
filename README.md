insturctions

adb push ~/Documents/tb3_controller /root/catkin_ws/src/
adb push ~/Documents/tb3_controller/nodes/turtlebot3_teleop_key /root/catkin_ws/src/tb3_controller/nodes/

export ROS_MASTER_URI='http://192.168.0.123:11311'


export TURTLEBOT3_MODEL=waffle_pi
chmod +x /root/catkin_ws/src/tb3_controller/nodes/turtlebot3_teleop_key
roslaunch tb3_controller tb3_controller.launch