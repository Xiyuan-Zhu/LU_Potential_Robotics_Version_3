LU_Potential_Robotics_version_3
This README is designed in a way that assumes that the host has installed Docker and has the latest version of f1tenth_gym_ros

1. Git clone the LU_Potential_Robotics_version_3 package from https://github.com/Lu-Potential/LU_Potential_Robotics.git 
into the subfolder /src/f1tenth_gym_ros-master/ of your catkin workspace.

2. Make .py an executable

3. Start a docker container

4. In a different terminal, catkin_make, source devel/setup.bash, and run:
    $roslaunch f1tenth_gym_ros 'lu_kinetic_version_3'.launch
