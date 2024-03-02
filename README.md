## Moving a ball on the plate 

## Demo
[![Watch the video](https://img.youtube.com/vi/xZprBYt4qMk/hqdefault.jpg)](https://www.youtube.com/watch?v=xZprBYt4qMk)

## Build 
./build.s

## Run 
# run program in real, first
roslaunch tum_ics_ur_robot_manager robot_script_manager_ur10.launch

# simulation 
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch ur10_controller ur10_effort_controller.launch
