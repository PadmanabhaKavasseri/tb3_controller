# Turtlebot Controller
## **Steps for RB5**  
### _Open three Terminals_  
**All Terminals**
```shell
adb shell
```
```shell
source ~/.bashrc && . /root/catkin_ws/devel/setup.bash
```
```shell
ifconfig
```
```shell
export ROS_MASTER_URI='http://192.168.0.123:11311'
```

**First Terminal**  
*Project Build*

```shell
chmod +x /root/catkin_ws/src/tb3_controller/nodes/turtlebot3_teleop_key
cd ~/catkin_ws && catkin_make
```

**Second Terminal**  
*Turtlebot3 Bringup*

```shell
export OPENCR_MODEL=waffle_pi
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```


**Third Terminal**  
*Turtlebot3 Controller*

```shell
export OPENCR_MODEL=waffle_pi
export TURTLEBOT3_MODEL=waffle_pi
roslaunch tb3_controller tb3_controller.launch
```

TODO add SSH information

____________________________________________________________________
## **ADB Cheat sheet**
*From Linux Desktop to RB5*

*Whole Folder*  
```shell
adb push ~/Documents/tb3_controller /root/catkin_ws/src/
```

*Teleop Python File*  
```shell
adb push ~/Documents/tb3_controller/nodes/turtlebot3_teleop_key /root/catkin_ws/src/tb3_controller/nodes/
```