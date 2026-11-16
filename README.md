# OnRobot_ROS2_Description

ROS 2 description of the OnRobot grippers: geometry (XACRO and meshes), one data file per gripper, an RViz
configuration and a launch file to look at a gripper.

The package describes geometry only. The `ros2_control` hardware of the grippers is described in
[OnRobot_ROS2_Driver](https://github.com/gabrinovas/OnRobot_ROS2_Driver).

## Grippers

| Name | Gripper | Kind |
|---|---|---|
| `2fg7` | 2FG7 parallel gripper | finger |
| `3fg15` | 3FG15 three-finger centric gripper | finger |
| `vgc10` | VGC10 vacuum gripper | vacuum |

Every gripper is one file in [config/grippers](config/grippers): its kind, its commanded joints with their
limits, the distance to its tool centre point and the file with its geometry. Nothing else in the package names
a gripper, so the list above is simply the list of files in that folder.

### Adding a gripper

1. `config/grippers/<name>.yaml`, like the existing ones.
2. `urdf/grippers/<name>.xacro`, with a macro called `onrobot_gripper_geometry` that takes `prefix` and
   `gripper` (the content of the YAML file) and creates `${prefix}onrobot_base_link` and `${prefix}gripper_tcp`.
3. Its meshes in `meshes/<name>/`.

The launch files of this package, of the driver and of the UR + OnRobot packages then offer the new name.

## Use

Build and source the workspace, then:

```sh
ros2 launch onrobot_description view_onrobot.launch.py onrobot_type:=3fg15
```

| Argument | Meaning |
|---|---|
| `onrobot_type` | gripper to show, one of the names above |
| `prefix` | prefix of the link and joint names |
| `ns` | namespace of the nodes |
| `gui` | `true`: move the joints from a window; `false`: publish them fixed |

The values used when an argument is not given are in
[config/onrobot_description.yaml](config/onrobot_description.yaml), not in the launch file.

### In another description

```xml
<xacro:include filename="$(find onrobot_description)/urdf/onrobot_macro.xacro"/>
<xacro:onrobot onrobot_type="2fg7" prefix=""/>
```

The macro creates the gripper hanging from `${prefix}onrobot_base_link`; attach that link to your robot.
[urdf/onrobot.urdf.xacro](urdf/onrobot.urdf.xacro) is a complete example. Both parameters are required.

## Python helpers

The package installs a small Python module used by the launch files of the three repositories:

| Module | What it gives |
|---|---|
| `onrobot_description.config_loader` | `load_config`, `require`, `package_file`: YAML loading that stops with a `[Fail-Fast]` error naming the missing file or key |
| `onrobot_description.grippers` | `gripper_names()`, `load_gripper(name)` |
| `onrobot_description.launch_arguments` | `declare_arguments`: launch arguments declared from a table of a YAML file |
| `onrobot_description.source_rules` | the checks every package runs on its own files: at most 200 lines, ASCII only |

## Meshes

The meshes in `meshes/` are simplified versions (visual within 0.5 mm of the original, collision within 2 mm).
The originals are kept in `meshes_original/`, which is not installed; its README lists every mesh.

## License

MIT, see [LICENSE](LICENSE). The meshes and the original XACRO files are derived from
[Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot),
also under the MIT license.

Tony Le (https://github.com/tonydle), Gabriel Novas (https://github.com/gabrinovas).
