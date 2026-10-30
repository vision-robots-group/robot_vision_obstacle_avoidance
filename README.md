# Robot Obstacle Detection and Avoidance

## Stage 1 - Object detection: 
- use LiDAR or OAK-D camera sensors 
- read incoming info 
- we make a histogram where each bin is a range of a few degrees 
- decide if a detected object is an obstacle 

## Stage 2 - obstacle avoidance: 
- if the obstacle is close, turn in a different direction

## Workspace Setup 
```bash
colcon build --symlink-install
```

# Sensors

## LiDAR
```bash
ros2 run object_detection_pkg controller
```

## Depth Camera
```bash
ros2 run object_detection_pkg depth_controller
```

```bash
ros2 run object_detection_pkg depth_controller --ros-args \
    -p depth_topic:=/SOME_DEPTH_TOPIC \
    -p camera_info_topic:=/SOME_CAMERA_INFO_TOPIC
```

### Depth Camera in Gazebo (with tunable parameters)
```bash
PYTHONNOUSERSITE=1 ros2 run object_detection_pkg depth_controller --ros-args \
    -p depth_topic:=/camera/depth/image_raw \
    -p camera_info_topic:=/camera/depth/camera_info \
    -p use_sim_time:=true \
    -p safe_distance:=0.8 -p forward_speed:=0.15 -p turn_speed:=0.4 -p fov_deg:=25.0 \
    -p turn_choice:=space
```