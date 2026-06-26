# ANTSY - Autonomous Navigator and Terrain Surveyor Yetti: a ROS 2 Hexapod Robot with MuJoCo Simulation
Yetti? Well, it is not a spider. It's more like an aberration, like a Yetti!

This package can be used for hexapod simulation in ROS2. The `mujoco-simulation` branch uses MuJoCo as the simulator backend.

## Demo

<a href="https://www.youtube.com/watch?v=J4Mk0Q0KOWg">
  <img
    src="https://img.youtube.com/vi/J4Mk0Q0KOWg/hqdefault.jpg"
    alt="ANTSY demo video"
    width="480">
</a>

<img
  src="antsy_sim_demo.gif"
  alt="ANTSY simulator demo"
  width="480">

## Instalation

- `git clone git@github.com:Laroto/antsy.git`
- `git submodule update --init --recursive`
- `docker compose build --pull`
- `docker compose run --rm antsy`
- `colcon build`
    - (optional) For convenience, we can instead run `colcon build --symlink-install` to create simlinks. This will allow to edit non-compiled files (eg: Python scripts or parameter files) without having to source the workspace again
- `source install/setup.bash`

## Running

There is more information about each component of this package in the submodules of this reporsitory:
- [src/antsy_description/README.md](src/antsy_description/README.md) - robot model, description launch files, and their launch arguments
- [src/antsy_simulation/README.md](src/antsy_simulation/README.md) - MuJoCo simulator node and simulation launch arguments
- [src/antsy_control/README.md](src/antsy_control/README.md) - control nodes, gait parameters, body-pose mode, and leg odometry
- [src/ds4_launcher/README.md](src/ds4_launcher/README.md) - DS4 launch setup, teleop config, and bridge-node parameters
- [src/hiwonder_ros2/README.md](src/hiwonder_ros2/README.md) - servo writer nodes and serial settings
- [src/antsy_kinematics/README.md](src/antsy_kinematics/README.md) - kinematics library and example node
- [src/antsy_msgs/README.md](src/antsy_msgs/README.md) - message-only package, no runtime parameters

There are more submodules/packages but they are used as libraries and/or dependencies so no need to worry about those ;)

## Parent repo scripts

The parent repo does not provide production ROS nodes. It provides helper scripts used for offline smoke testing:

- `scripts/smoke_control_pipeline.py`
- `scripts/smoke_sim_leg_odometry.py`
- `scripts/smoke_hiwonder_writer.py`
- `scripts/run_offline_smoke_tests.sh`

These scripts use environment variables rather than ROS parameters.

Relevant environment variables:

| Variable | Default | Meaning |
| --- | --- | --- |
| `ANTSY_TEST_ROS_DOMAIN_ID` | `93` | ROS domain used by the offline smoke suite so it stays isolated from other ROS graphs. |
| `ANTSY_TEST_LOG_DIR` | `/tmp/antsy_smoke_tests` | Directory where the smoke scripts store subprocess logs. |
| `ANTSY_SIM_SMOKE_LINEAR_X` | `0.25` | Forward speed used by the headless simulation odometry smoke test. |
| `ANTSY_SIM_SMOKE_WARMUP` | `2.0` | Warmup time, in simulated seconds, before the steady-state leg-odometry window is measured. |
| `ANTSY_SIM_SMOKE_DURATION` | `10.0` | Simulated duration used by the headless simulation odometry smoke test. |
| `ANTSY_SIM_SMOKE_COOLDOWN` | `2.0` | Time, in simulated seconds, spent publishing zero `cmd_vel` after the motion window so the gait can settle before the final pose comparison. |
| `ANTSY_SIM_SMOKE_MAX_FORWARD_RELATIVE_ERROR` | `0.01` | Maximum allowed relative error between simulator and leg-odometry forward-motion trends over the analysis window. |
| `ANTSY_SIM_SMOKE_MAX_HEADING_ERROR` | `0.03` | Maximum allowed integrated heading-trend mismatch between simulator and leg odom over the analysis window. |
| `ANTSY_SIM_SMOKE_MAX_COOLDOWN_POSITION_ERROR` | `0.03` | Maximum allowed XY pose mismatch between simulator odom and leg odom after the cooldown period. |
| `ANTSY_SIM_SMOKE_MAX_COOLDOWN_HEADING_ERROR` | `0.03` | Maximum allowed yaw mismatch between simulator odom and leg odom after the cooldown period. |
| `ANTSY_SIM_SMOKE_MIN_FORWARD` | `1.80` | Minimum required simulator forward motion during the headless simulation test. |
| `ANTSY_SIM_SMOKE_MAX_LATERAL` | `0.15` | Maximum allowed simulator lateral drift during the headless simulation test. |
| `ANTSY_SIM_SMOKE_MAX_LEG_LATERAL` | `0.08` | Maximum allowed leg-odometry lateral drift during the headless simulation test. |
| `ANTSY_SIM_SMOKE_LEG_ODOM_TRANSLATION_SCALE` | `1.0` | Translation calibration passed into `leg_odometry.translation_scale` during the steady-state odometry smoke test. Keep this at `1.0` unless you are deliberately testing a calibrated bias correction. |
| `ANTSY_SIM_SMOKE_LEG_ODOM_PROPAGATE` | `0` | If true, enables short leg-odometry dead-reckoning during invalid support windows for this smoke test only. |
| `ANTSY_SIM_SMOKE_LEG_ODOM_MAX_PREDICTION_TIME` | `0.08` | Maximum dead-reckoning time used when `ANTSY_SIM_SMOKE_LEG_ODOM_PROPAGATE` is enabled. |
| `ANTSY_SIM_SMOKE_WALL_TIMEOUT` | `40.0` | Wall-time timeout for the headless simulation test to advance the requested simulation duration. |

## Offline smoke tests

Run the real-robot command pipeline checks without a remote controller or real servos:

```bash
scripts/run_offline_smoke_tests.sh
```

The smoke suite runs inside Docker, builds the required packages, and uses an isolated ROS domain by default. Override it with:

```bash
ANTSY_TEST_ROS_DOMAIN_ID=94 scripts/run_offline_smoke_tests.sh
```

The tests check:

- `cmd_vel` through `antsy_description` + `antsy_control` produces changing `/actuators` commands.
- `/leg_odom` moves forward under forward `cmd_vel`, stays within a small lateral drift bound, and resets near the origin after `/control/reset`.
- Headless MuJoCo simulation runs in open loop with heading hold disabled, warms up for 2 simulated seconds, drives forward for 10 simulated seconds, then publishes zero `cmd_vel` for 2 simulated seconds before comparing the settled final pose. During the motion window it compares the steady-state forward and heading trends of simulator `/odom` against controller `/leg_odom`. In simulation, `/leg_odom` uses measured `/joint_states`; without measured joints it falls back to commanded foot poses.
- `/control/reset` returns the controller output close to the startup actuator command.
- `hiwonder_ros2/write_only` can run against a pseudo-terminal, receives `/actuators`, writes serial move packets, sends servo unload on `/motors/disable`, pauses writes while disabled, and resumes on `/motors/enable`.

`/actuators` is not simulator-only. It is the controller output topic. The MuJoCo simulator subscribes to it in simulation, and the Hiwonder writer subscribes to it on the real robot.
