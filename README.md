# stymphale_project

The end goal of this project is to create a ROS2 package for 3D SLAM with decentralized autonomous drone swarm (point-cloud based, potentially multi-sensors ex: thermal). \
The project is divided between severals "modules" :

- Zawardo_drone : Docker environement for coding, test and simulation
- Vox_drone : Python module. Experiment voxel maps possibilities
- Antikythera : ROS package. SLAM algoritm

## Zawardo_drone

The purpose of the zawardo_drone module is to create a docker environement for drone swarm simulation.
The environement include :

- Gazebo Harmonic
- ROS2 Humble
- Ardupilot
- SITL (Ardupilot Software In The Loop)
- Ardupilot ROS2 and Gazebo bridge
- QGRoundControl
- MAVPROXY

And some other tools like :

- terminator
- vim
- nano
- git

<https://ardupilot.org/dev/docs/ros2-sitl.html>

## Antikythera

## Vox_drone

Vox drone was a data structure test to adress the problem of the map exchange between the drones, using mostly voxels and FFT.
This module isn't actively developed anymore, until I have to deal with this problem with other modules.

## Documentation

Open3D Library : <https://www.open3d.org/docs/release/index.html>

Ardupilot : <https://ardupilot.org/dev/index.html>
