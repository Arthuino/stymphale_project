# ZAWARDO_DRONE

## Taking in hand Ardupilot/MAVROS

### Launch basic simulation

QGroundControl software

```bash
qgroundcontrol-start
```

SITL with ros2 and gazebo

```bash
ros2 launch ardupilot_sitl sitl_mavproxy.launch.py console:=True map:=True
```

### Basic takeoff and control

In the mavproxy logs, wait for the drone to connect his EKF to GPS.
Then, in terminal :

```bash
mode GUIDED
arm throttle
takeoff 10
```

### QGroundControl

Create a new connection in Application Settings>Comm Links
Give a name, set type to TCP and set the drone IP and port (8100).

Click one the map to set waypoints.

## Docker zawardo

Pull

```bash
docker pull arthuino/stymphale-zawardo:latest
```

Run

```bash
./.config/docker_run.sh
```

Build

```bash
./.config/docker_build.sh
```

Push

```bash
docker push arthuino/stymphale-zawardo:latest
```

root password inside docker : ``zawardo``

### Usage

SITL (Software In The Loop) simulation of an Ardupilot drone.
The console option give MAVROS logs in a separate terminal with more infos.
The out option give a TCP connection to the drone (usefull for QGroundControl).
The map option give a map to give order to the drone (can "repalce" QGC)
This cmd will create files where it is called.

```bash
sim_vehicle.py -v [VEHICULE] -f [FRAME] -I0 --console --map --out=tcpin:[IP]:[PORT]
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console --map --out=tcpin:0.0.0.0:8100
```

SITL launch trough ros2

```bash
ros2 launch ardupilot_sitl sitl_mavproxy.launch.py console:=True map:=True
```

Launch ros2 + sitl + mavproxy + gazebo + rviz

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

Gazebo alone

```bash
gz sim
```

Connect Mavproxy console to running SITL

```bash
mavproxy.py --console --map --aircraft test --master=:14550
```

QGroundControl

```bash
qgroundcontrol-start
```
