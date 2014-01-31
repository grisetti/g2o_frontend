#launch the turtlebot, minimal configuration
roslaunch turtlebot_bringup minimal.launch

roslaunch hokuyo_node hokuyo_test.launch

rosrun tf static_transform_publisher 0.0 0.1 0.2 0.0 0.0 0.0 base_link laser 100

#@home
rosrun tf static_transform_publisher 0 0.2 0.2 0 0 0 /base_link /laser 100

#@work
rosrun tf static_transform_publisher 0 0.2 0.0 0 0 0 /base_link /laser 100

cd ~/catkin_ws
source ./devel/setup.bash
roslaunch freenect_camera freenect_turtlebot.launch 
rosbag record /clock /tf /odom /scan /camera/depth_registered/camera_info /camera/depth_registered/image_rect_raw /camera/rgb/image_rect /camera/rgb/camera_info
