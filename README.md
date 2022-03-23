# Drone Project
Execute these commands to run the whole integrated system

## Brain
```
roslaunch dd2419_launch base.launch ch:=96
rviz
rqt &
roslaunch dd2419_launch world_publisher.launch world_name:=saal2
rosrun part2 navgoal3
rosrun flight_camp roundtrip_saal2
```

## Localization
```
rosrun localization static 
rosrun localization loc2
```

## Perception
```
rosrun perception sign_detection.py
```

## Pathplanning
tbf.
```
rosrun planning path_planner
```

