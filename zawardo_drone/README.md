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

Based on ardupilot ROS2 tutorial :
<https://ardupilot.org/dev/docs/ros2.html>

## How to use

### With docker compose

Build ros packages

```bash
docker compose up cpp_ros_builder
```

Launch micro-ros agent

```bash
docker compose up micro_ros_agent
```

Launch QGroundControl

```bash
docker compose up qgcontrol
```

Simulation Docker environement

```bash
docker compose run ardupilot_ros_env
```

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

1. Run Micro-ROS agent : \

    ```bash
    docker compose up micro_ros_agent
    ```

2. Run the SITL, either with ```sim_vehicle``` or trough ROS2, without or with Gazebo : \
    In the ardupilot environement docker :

    ```bash
    docker compose run --remove-orphans ardupilot_ros_env 
    ```

    a. with ```sim_vehicle``` :

    ```bash
    sim_vehicle.py -w -v [VEHICULE] -DG --enable-dds --out=tcpin:[IP]:[PORT] --console --map
    ```

    Example with ArduCopter :

    ```bash
    sim_vehicle.py -w -v ArduCopter -DG --enable-dds --out=tcpin:0.0.0.0:8100  --console --map
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
    docker compose up qgroundcontrol
    ```

4. Connect a MAVPROXY console to already running SITL simulation launched with ros launch :

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

## Use AP_DDS ros services

### Takeoff

Switch to guided mode [doc for copter](https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE)

```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "mode: 4"
```

Arming motors

```bash
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "arm: true"
```

Takeoff

```bash
ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "alt: 5.0"
```

Land

```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "mode: 9"
```

## Other Useful Commands and infos

Build docker image

```bash
./.config/docker_build.sh
```

Push docker image

```bash
docker push arthuino/stymphale-zawardo:latest
```

Launch Gazebo alone

```bash
gz sim
```

non-root user name : ``ardupilotuser`` \
root password inside docker : ``zawardo``

## Future improvements

- [X] Test waypoints and mission planning with ROS2 services and nodes

- [X] Create separate dockers and docker-compose for each tool
