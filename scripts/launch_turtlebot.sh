#launch the turtlebot, minimal configuration
roslaunch turtlebot_bringup minimal.launch
roslaunch hokuyo_node hokuyo_test.launch
roslaunch human_interface robot_teleop_joy.launch

cd ~/catkin_ws
source ./devel/setup.bash
roslaunch freenect_camera freenect_turtlebot.launch 
rosrun tf static_transform_publisher 0 0.2 0.2 0 0 0 /base_link /laser 100
rosbag record /clock /tf /odom /scan /camera/depth_registered/image_rect_raw