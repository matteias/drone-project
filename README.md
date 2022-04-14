# Drone Project
Execute these commands to run the whole integrated system

to find camera: 
'''
cvlc v4l2:///dev/v4l/by-id/usb-ARKMICRO_USB2.0_PC_CAMERA-video-index0 :live-caching=25
'''
new world:
'''
rosrun dd2419_simulation json_to_world.py awesome.world
'''
Alternatively, use launch files for the whole system and run perception and localization node seperately
```
roslaunch brain ms3.launch
roscd perception/scripts
rosrun perception perception.py
rosrun localization loc4_signs
```

## Brain
```
roslaunch dd2419_launch base.launch ch:=96
rviz
rqt &
roslaunch dd2419_launch world_publisher.launch world_name:=saal3
rosrun part2 navgoal3
```

## Localization
```
rosrun tf2_ros static_transform_publisher 0.01 0 0.02 -1.57 0 -1.57 cf1/base_link cf1/camera_link
rosrun localization static
rosrun localization loc4_signs
```

## Perception
dependecies: PyTorch (python3)

```
roscd perception/scripts

rosrun perception perception.py
rosrun perception trainsforms.py
rosrun perception intruder.py
```
Perception runs sign detection and publishes detected markers as markerarrays.
Trainsforms publishes detected signs as transforms /perception/detectedX
and signs from the map as /perception/signX.

<img src="pose_estimation.png" alt="pose_estimation" width="300"/>

## Pathplanning
map figure:
'''
rosrun pathplanning map_creator
'''
animation of path
```
rosrun Pathplanning a_star.py
```
