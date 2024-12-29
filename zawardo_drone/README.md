# ZAWARDO_DRONE

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

## How to use

### Docker environement

Pull the docker image :

```bash
docker pull arthuino/stymphale-zawardo:latest
```

Run the docker :

```bash
./.config/docker_run.sh
```

### Run simulation

In the seperate docker terminals :

1. Run Micro-ROS agent : \
-p specify the udp port used

    ```bash
    ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
    ```

2. Run the SITL, either with ```sim_vehicle``` or trough ROS2, without or with Gazebo :

    a. with ```sim_vehicle``` :

    ```bash
    sim_vehicle.py -w -v [VEHICULE] -f [FRAME] -DG --enable-dds -I[SITL instance ID] --out=tcpin:[IP]:[PORT] --console --map
    ```

    Example with ArduCopter :

    ```bash
    sim_vehicle.py -w -v ArduCopter -f gazebo-iris -DG --enable-dds -I0 --out=tcpin:0.0.0.0:8100  --console --map
    ```

    b. with ROS2 launch :

    ```bash
    ros2 launch ardupilot_sitl sitl_mavproxy.launch.py console:=True map:=True
    ```

    c. with ROS2 launch and Gazebo :

    ```bash
    ros2 launch ardupilot_gz_bringup iris_runway.launch.py
    ```

3. Launch QGroundControl if you wish to use it : \
Create a new connection in Application Settings>Comm Links \
Give a name, set type to TCP and set the drone IP and port (ex: 8100)

    ```bash
    qgroundcontrol-start
    ```

4. Connect a MAVPROXY console to already running SITL simulation :

    ```bash
    mavproxy.py --console --map --aircraft test --master=:14550
    ```

### Basic takeoff and control

In the mavproxy logs, wait for the drone to connect his EKF to GPS.
Then, in terminal :

```bash
mode GUIDED
arm throttle
takeoff 10
```

Control the drone in mavproxy

```bash
velocity x y z
```

### Built-in SLAM and navigation tools

- Maze simulation in Gazebo. \
Include a simple maze and a drone with a 2D Lidar.

    ```bash
    ros2 launch ardupilot_gz_bringup iris_maze.launch.py
    ```

- SLAM cartographer \
Create a map based on the lidar data.

    ```bash
    ros2 launch ardupilot_ros cartographer.launch.py
    ```

    save map of the cartographer

    ```bash
    ros2 run nav2_map_server map_saver_cli -f /path/to/directory/map_name
    ```

## Other Useful Commands

Build docker image

```bash
./.config/docker_build.sh
```

Push docker image

```bash
docker push arthuino/stymphale-zawardo:latest
```

root password inside docker : ``zawardo``

Launch Gazebo alone

```bash
gz sim
```

## Future improvements

- Improve portability (volumes)
