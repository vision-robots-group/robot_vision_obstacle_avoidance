# robot_vision_obstacle_avoidance

SASSY ROBOT 

## stage 1 - Object detection: 
- use LiDAR or OAK-D camera sensors 
- read incoming info and use threshold to detect objects 
- decide if a detected object is an obstacle 

## Stage 2 - obstacle avoidance: 
- if the obstacle is close, turn in a different direction (random generated angle b/w 60 and 180 deg)



# SETup workspace 
colcon build --symlink-install
