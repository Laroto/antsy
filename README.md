# ANTSY - Autonomous Navigator and Terrain Surveyor Yetti
Yetti? Well, it is not a spider. It's more like an aberration, like a Yetti!

This package can be used for hexapod simulation in ROS2. The `mujoco-simulation` branch uses MuJoCo as the simulator backend.

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
- [antsy_description](https://github.com/Laroto/antsy_description/tree/main) - handles the robot model related things. It's necessary for pretty much everything else
- [antsy_simulation](https://github.com/Laroto/ds4_launcher/tree/main) - contains everything simulation related. No need to use this if using the real ANTSY
- [antsy_control](https://github.com/Laroto/antsy_control/tree/main) - handles all the control. Mandatory for all uses (unless you don't want our Yetti to move).
- [ds4_launcher](https://github.com/Laroto/ds4_launcher/tree/main) - just a simple convenience tool to use a PS4 controller to remote control our Yetti.

There are more submodules/packages but they are used as libraries and/or dependencies so no need to worry about those ;)
