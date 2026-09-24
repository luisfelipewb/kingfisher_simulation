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
ros2 launch kingfisher_sim kingfisher_sim.launch.py
```

Spawn configs name it as `model_type: usv` + `urdf: package://kingfisher_gazebo/urdf/kingfisher_gazebo.urdf.xacro`
(see `kingfisher_sim/config/kingfisher_spawn.yaml`).
