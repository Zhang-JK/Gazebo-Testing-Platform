# Gazebo Testing Platform

## How to use
1. Install the following packages:  
    - gazebo_ros
    - turtlebot3_navigation
    - turtlebot3_gazebo
    - gmapping
    - rviz
    - robot_localization
    - joy
    - teleop_twist_keyboard
    - map_server  

2. (Optional) Create a map in Gazebo and use [```gazebo_ros_2Dmap_plugin```](https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin) to generate a map file.
3. Finish the grid command file at ```src/move_grid/command/movement.txt```, the robot will follow the commands in the file. If you are using a external map, you need to change the map file path in ```src/move_grid/config/params.yml```.
4. Compile the program with ```catkin_make``` and source the config file ```devel/setup.bash```.
4. Launch the program with ```roslaunch move_grid move.launch```.