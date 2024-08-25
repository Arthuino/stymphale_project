# ZAWARDO_DRONE

## Taking in hand Ardupilot/MAVROS

### Launch basic simulation

QGroundControl software

```bash
./QGroundControl.AppImage
```

Gazebo simulation

```bash
gazebo --verbose ardupilot_gazebo/worlds/iris_ardupilot.world
ou
rosrun gazebo_ros gazebo ardupilot_gazebo/worlds/iris_ardupilot.world
ou
roslaunch ardupilot_gazebo ardupilot.launch
```

MAVROS of an Ardupilot drone.
The console option give MAVROS logs in a separate terminal with more infos.
The out option give a TCP connection to the drone (usefull for QGroundControl).

```bash
sim_vehicle.py -v [VEHICULE] -f [FRAME] -I0 --console --out=tcpin:[IP]:[PORT]
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console --out=tcpin:0.0.0.0:8100
```

Wait for the MAVROS to connect his IMU to GPS.

- Manual takeoff

```bash
mode GUIDED
arm throttle
takeoff 10
```

- QGroundControl

Create a new connection in Application Settings>Comm Links
Give a name, set type to TCP and set the drone IP and port (8100).

Click one the map to set waypoints.

MAVROS 

```bash
roslaunch mavros px4.launch fcu_url:=tcp://localhost:5760
```

