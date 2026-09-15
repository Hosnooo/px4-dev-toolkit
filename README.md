# px4-dev-toolkit

A reproducible development toolkit for PX4 simulation, ROS 2 integration, experiment execution, flight-data recording, and control-pipeline analysis.

The goal is to provide a practical PX4 development environment that can be reproduced on another workstation and extended into ready-to-run Docker images for simulation and experimental platforms.

## Current capabilities

- PX4 SITL with Gazebo
- ROS 2 Jazzy integration through uXRCE-DDS
- pinned external source revisions for reproducibility
- MAVProxy and QGroundControl support
- reusable SITL launcher
- ROS 2 runtime environment
- automatic rosbag recording
- centralized PX4 ROS topic definitions
- generic PX4 multicopter control-pipeline analysis
- publication-style tracking plots
- layer-by-layer run summaries

The currently validated host platform is Ubuntu 24.04 (Noble) with ROS 2 Jazzy.

## Repository layout

```text
px4-dev-toolkit/
├── analysis/        PX4 flight-data analysis
├── config/          runtime configuration and PX4 topic definitions
├── px4/             PX4 build helpers
├── ros2/            ROS 2 workspace, runtime environment, and packages
├── setup/           installation and dependency setup
├── tools/           user-facing launch and analysis tools
└── px4_env.repos    pinned external source repositories
```

External projects such as PX4-Autopilot, `px4_msgs`, `px4_ros_com`, Micro-XRCE-DDS-Agent, and MAVProxy are fetched at pinned revisions and are not vendored into this repository.

## Setup

Clone the repository:

```bash
git clone https://github.com/Hosnooo/px4-dev-toolkit.git
cd px4-dev-toolkit
```

Install the development environment:

```bash
./setup/install.sh
```

External source revisions are defined in:

```text
px4_env.repos
```

Validated host and tool versions are documented in:

```text
config/versions.env
```

## Build the ROS 2 workspace

```bash
./ros2/build.sh
```

Before running ROS 2 commands manually, source the runtime environment:

```bash
source ros2/runtime_env.sh
```

## Run PX4 SITL

Start the configured PX4 SITL environment with:

```bash
./tools/sitl
```

Runtime settings are defined in:

```text
config/env.env
```

The current default configuration uses:

- PX4 SITL
- Gazebo
- `gz_f450`
- the default Gazebo world
- uXRCE-DDS over UDP
- MAVProxy as the default ground-control frontend

The ground-control frontend can be configured to use MAVProxy, QGroundControl, or none.

## PX4 control-pipeline analysis

Analyze a recorded run with:

```bash
./tools/px4_analyze bags/<experiment>/<run>
```

The analyzer follows the native PX4 multicopter control pipeline using signals exposed through ROS 2.

The analyzed chain includes:

```text
trajectory_setpoint
        ↓
vehicle_local_position_setpoint
        ↓
vehicle_local_position

vehicle_attitude_setpoint
        ↓
vehicle_attitude

vehicle_rates_setpoint
        ↓
vehicle_angular_velocity

vehicle_thrust_setpoint
vehicle_torque_setpoint
        ↓
actuator_motors
```

Standard analysis output:

```text
01_position_tracking.png
02_velocity_tracking.png
03_acceleration_tracking.png
04_attitude_tracking.png
05_rate_tracking.png
06_thrust_setpoint.png
07_torque_setpoint.png
08_actuator_motors.png
09_mode_timeline.png
summary.txt
```

Tracking plots compare independently timestamped PX4 signals and include component-wise error plots where an actual/reference pair is available.

The generated summary reports the control pipeline layer by layer:

1. trajectory reference
2. position controller
3. attitude controller
4. rate controller
5. controller output
6. control allocation

It includes signal counts, ranges, RMS tracking errors, peak errors, thrust and torque statistics, and motor allocation statistics.

## Centralized PX4 topics

PX4 ROS topics used by the toolkit are defined in one project-wide catalog:

```text
config/px4_topics.def
```

The catalog is shared by:

- ROS 2 C++ experiment code
- the Python analyzer
- rosbag recording configuration

This avoids duplicating `/fmu/in/*` and `/fmu/out/*` topic names throughout the project.

## Reproducibility

The environment uses pinned external repositories rather than copying their source into this repository.

Current external dependencies include:

- PX4-Autopilot
- `px4_msgs`
- `px4_ros_com`
- Micro-XRCE-DDS-Agent
- MAVProxy

Their exact revisions are recorded in:

```text
px4_env.repos
```

The PX4 revision used by this toolkit includes additional DDS publications required to observe the native controller pipeline from ROS 2.

## Project direction

This repository is intended to grow into a general PX4 development toolkit rather than a single experiment.

Planned additions include:

- Dockerized development environments
- ready-to-run simulation Docker images
- Docker images for experimental platforms
- additional PX4 experiment profiles
- reusable controller examples
- additional vehicles and Gazebo configurations
- ready-to-run experiment scripts
- hardware experiment workflows
- expanded logging and analysis
- controller and experiment comparison tools

The shared environment, launch infrastructure, recording tools, and analysis pipeline are intended to remain reusable while individual experiments stay isolated.

## License

Toolkit code in this repository is released under the BSD 3-Clause License.

Fetched third-party projects retain their respective upstream licenses.