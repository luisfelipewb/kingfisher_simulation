# kingfisher_simulation

Gazebo (gz-sim) / VRX simulation of the Clearpath Kingfisher ASV, ROS 2 Jazzy.

| package | role |
|---|---|
| `kingfisher_gazebo` | gz-sim layer: hydrodynamics, thrusters, sensors. `urdf/kingfisher_gazebo.urdf.xacro` is what VRX spawns |
| `kingfisher_sim` | launch + `cmd_drive_translate` (normalized `cmd_drive` → thrust in N) |

Depends on VRX and on `kingfisher_msgs` and `kingfisher_description` from
[`kingfisher`](https://github.com/luisfelipewb/kingfisher); clone it into the same
workspace. `kingfisher_gazebo` adds the plugins and sensors to that URDF, so the
sim and the boat share one set of frames.

```
ros2 launch kingfisher_sim kingfisher_sim.launch.py                        # Lac Symphonie
ros2 launch kingfisher_sim kingfisher_sim.launch.py world:=sydney_regatta  # any VRX world
```

That is a world plus `robot.launch.py`, the sim twin of the boat's `kingfisher_bringup`
`robot.launch.py`: `kingfisher_twist` and `kingfisher_viz` with the boat's config, and
`cmd_drive_translate` in place of the MCU. It also runs on its own, once per Kingfisher, after
any world that spawns one (`name:=<model name>`).

Spawn configs name it as `model_type: usv` + `urdf: package://kingfisher_gazebo/urdf/kingfisher_gazebo.urdf.xacro`
(see `kingfisher_sim/config/kingfisher_spawn.yaml`).

`config/joy_teleop.yaml` sends Newtons straight to `thrusters/*`, bypassing `cmd_drive_translate`.
It is for debugging only; normal teleop goes through `cmd_drive` or `cmd_vel`, as on the boat.
