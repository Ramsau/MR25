## Setting up ROS2

- System requirements: Ubuntu 22.04
- Install ROS2 Humble as described in the [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Install the full desktop version (`ros-humble-desktop-full`).
- Export the following variables
  - `ROS_LOCALHOST_ONLY=1`
  - `ROS_DOMAIN_ID=0`

## Setting up the workspace

- Create a new worspace folder
  - `mkdir -p ~/ros2_ws/src`
  - `cd ~/ros2_ws`
- Build the workspace
  - `colcon build`
- Source the workspace
  - `source install/local_setup.bash`

## Setting up the assignment repository

- Clone this repository
- Link the packages to your workspace
  - `ln -s PATH/TO/CLONED/REPO/tug_stage_ros2 ~/ros2_ws/src`
  - ...
- Build the workspace
- Source the workspace
