# Drone Project
## Localization

The commands for localization node: 
```
roslaunch dd2419_launch base.launch ch:=96
transform tf for camera_link to base_link
rosrun localization static
rviz 
rosrun localization loc2
```
## Perception

## Pathplanning
The commands for map to grid:
Remember to select the write path and size
```
rosrun pathplanning map_creator
```
## Brain
