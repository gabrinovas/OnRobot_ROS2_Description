# OnRobot_ROS2_Description

A ROS2 URDF description package for OnRobot grippers. This package provides XACRO macros, meshes, an RViz configuration and a simple launch file to visualise the gripper models.

## Contents

- Package manifest and build: [CMakeLists.txt](CMakeLists.txt)
- Launch files: [launch/view_onrobot.launch.py](launch/view_onrobot.launch.py), [launch/view_onrobot.launch_bk.py](launch/view_onrobot.launch_bk.py)
- URDF / XACRO: [urdf/onrobot_macro.xacro](urdf/onrobot_macro.xacro), [urdf/onrobot.urdf.xacro](urdf/onrobot.urdf.xacro)
- RViz config: [rviz/view_onrobot.rviz](rviz/view_onrobot.rviz)
- Meshes: [meshes/](meshes/)
- License: [LICENSE](LICENSE)

## Supported gripper types

The launch file supports these gripper types:
- rg2
- rg6
- 2fg7
- 3fg15

(See the `onrobot_type` launch argument in [launch/view_onrobot.launch.py](launch/view_onrobot.launch.py).)

## Installation

1. Clone into your workspace `src` directory:

   ```sh
   git clone https://github.com/tonydle/OnRobot_ROS2_Description.git src/onrobot_description
   ```

2. Build with colcon (symlink install recommended for easier development):

   ```sh
   colcon build --symlink-install
   ```

3. Source the workspace:

   ```sh
   source install/setup.bash
   ```

## Quick test (visualisation)

Launch RViz2 with an example gripper:

```sh
ros2 launch onrobot_description view_onrobot.launch.py onrobot_type:=rg2
```

This launch:
- Generates a URDF from [urdf/onrobot_macro.xacro](urdf/onrobot_macro.xacro) via xacro,
- Starts `joint_state_publisher_gui` and `robot_state_publisher`,
- Opens RViz2 with [rviz/view_onrobot.rviz](rviz/view_onrobot.rviz).

Use `onrobot_type:=rg2|rg6|2fg7|3fg15`. `prefix:=` and `ns:=` are available for multi-robot setups (see [launch/view_onrobot.launch.py](launch/view_onrobot.launch.py)).

## How to use the XACRO macro

Include the macro in your robot URDF and attach the `onrobot_base_link` where required:

```xml
<xacro:include filename="$(find onrobot_description)/urdf/onrobot_macro.xacro"/>
<xacro:onrobot onrobot_type="$(arg onrobot_type)" prefix="$(arg prefix)"/>
...
<link name="world" />
<joint name="$(arg prefix)onrobot_base_link_joint" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <parent link="world"/>
  <child link="$(arg prefix)onrobot_base_link"/>
</joint>
```

An example usage is provided in [urdf/onrobot.urdf.xacro](urdf/onrobot.urdf.xacro).

## Development notes

- Launch files call `xacro` via a Command substitution to generate `robot_description` at runtime. See [launch/view_onrobot.launch.py](launch/view_onrobot.launch.py).
- A backup variant of the launch file is present at [launch/view_onrobot.launch_bk.py](launch/view_onrobot.launch_bk.py).

## License

This project is released under the MIT License — see [LICENSE](LICENSE). The meshes and original xacros are derived from the [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot) project, also MIT-licensed.

## Author / Contact

The original mesh files and xacros are derived from [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot), which is licensed under the MIT License. \
Tony Le — https://github.com/tonydle \
Gabriel Novas — https://github.com/gabrinovas