# stymphale_project

Project of drone swarm.

The purpose of this project is to test several algorithms, strategies and tools trough differents "modules", in the idea of creating a swarm of drones.

The project also have a self-training goal.

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
-nano
- git

<https://ardupilot.org/dev/docs/ros2-sitl.html>

## Vox_drone

Vox drone was a data structure test to adress the problem of the map exchange between the drones, using mostly voxels and FFT.
This module isn't actively developed anymore, until I have to deal with this problem with other modules.

## Documentation

Open3D Library : <https://www.open3d.org/docs/release/index.html>

Ardupilot : <https://ardupilot.org/dev/index.html>
