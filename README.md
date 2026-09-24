# kingfisher_simulation

Gazebo (gz-sim) / VRX simulation of the Clearpath Kingfisher ASV, ROS 2 Jazzy.

| package | role |
|---|---|
| `kingfisher_description` | URDF/xacro and meshes, no Gazebo. Moves to [`kingfisher`](https://github.com/luisfelipewb/kingfisher) |
| `kingfisher_gazebo` | gz-sim layer: hydrodynamics, thrusters, sensors. `urdf/kingfisher_gazebo.urdf.xacro` is what VRX spawns |
| `kingfisher_sim` | launch + `cmd_drive_translate` (normalized `cmd_drive` → thrust in N) |

Depends on `kingfisher_msgs` from `kingfisher` and on VRX.

```
ros2 launch kingfisher_sim kingfisher_sim.launch.py
```

Spawn configs name it as `model_type: usv` + `urdf: package://kingfisher_gazebo/urdf/kingfisher_gazebo.urdf.xacro`
(see `kingfisher_sim/config/kingfisher_spawn.yaml`).

`kingfisher_description` is imported from Clearpath's
[kf/kingfisher](https://github.com/kf/kingfisher) @ `indigo-devel` `c7fb559`.
