# Antikythera

The purpose of this module is to work on the implementation of a SLAM algorithm.

## Usage

Rebuild builder Docker (ex: after adding new dependencies)

Build Stymphale workspace

```bash
docker compose up --remove-orphans cpp_ros_builder
```

```bash
docker compose build --remove-orphans cpp_ros_builder
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
docker compose up --remove-orphans cpp_ros_tester
```

View test results

```bash
docker compose up --remove-orphans cpp_ros_test_results
```

## Run

```bash
ros2 run antikythera antikythera_land_mark_slicer
```
