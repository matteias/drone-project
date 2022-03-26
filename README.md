# Drone Project
Execute these commands to run the whole integrated system

## Brain
```
roslaunch dd2419_launch base.launch ch:=96
rviz
rqt &
roslaunch dd2419_launch world_publisher.launch world_name:=saal2
rosrun part2 navgoal3
```

## Localization
```
rosrun localization static
rosrun localization loc3_sameID
```

## Perception
<img src="pose_estimation.png" alt="pose_estimation" width="150"/>
```
rosrun perception sign_detection.py
```

## Pathplanning
animation of path
```
rosrun Pathplanning a_star.py
```
