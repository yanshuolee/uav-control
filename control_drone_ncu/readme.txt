1. Open camera. 
roslaunch realsense2_camera rs_d400_and_t265.launch

2. Flight control.
roslaunch mavros apm2.launch 

3. T265 data to flight control.
rosrun tomavros tomavros

code: /home/nx/catkin_ws/src/tomavros/src

This program will not show anything.

4. set start point. Under ~/control_drone_ncu.
python3 set_origin_local.py

Check mission planner whether the plane show up.

5. Check if T265 is noraml? (Local linux computer)
roslaunch display_trajectory display_marker.launch


run python main.py code to start guide mode.
When the plane takes off, set remotor to on (F) and make acc to center



F: Arm/Disarm 
G: 


wifi: RTF/rsl@0912
nx/000000
