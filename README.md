# robot_vision_obstacle_avoidance

SASSY ROBOT 

## stage 1 - Object detection: 
- use LiDAR or OAK-D camera sensors 
- read incoming info 
- we make a histogram where each bin is a range of a few degrees 
- decide if a detected object is an obstacle 

## Stage 2 - obstacle avoidance: 
- if the obstacle is close, turn in a different direction



# SETup workspace 
colcon build --symlink-install


# LiDAR
ros2 run object_detection_pkg controller

# depth-camera 
ros2 run object_detection_pkg depth_controller


ros2 run object_detection_pkg depth_controller --ros-args \
    -p depth_topic:=/SOME_DEPTH_TOPIC \
    -p camera_info_topic:=/SOME_CAMERA_INFO_TOPIC