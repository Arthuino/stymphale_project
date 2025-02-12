# Antikythera

The purpose of this module is to work on the implementation of a SLAM algorithm.

## Usage

Rebuild builder Docker (ex: after adding new dependencies)

```bash
docker compose build cpp_ros_builder
```

Build Stymphale workspace

```bash
docker compose up cpp_ros_builder
```

Micro-ROS Agent

```bash
docker compose up micro_ros_agent
```

Simulation Docker environement

```bash
docker compose run ardupilot_ros_env
```

## Testing

Run tests

```bash
docker compose build cpp_ros_tester
```

View test results

```bash
colcon test-result --all --verbose
```

## Run

```bash
ros2 run antikythera antikythera_node
```
