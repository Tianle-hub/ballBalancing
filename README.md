./build.sh

## Run 
# run program in real, first
roslaunch tum_ics_ur_robot_manager robot_script_manager_ur10.launch

# simulation 
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch ur10_controller ur10_effort_controller.launch
