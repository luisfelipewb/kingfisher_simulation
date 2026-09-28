# kingfisher_simulation

Gazebo (gz-sim) / VRX simulation of the Clearpath Kingfisher ASV, ROS 2 Jazzy.

| package | role |
|---|---|
| `kingfisher_gazebo` | gz-sim layer: hydrodynamics, thrusters, sail, sensors, and the `ApparentWind` system. `urdf/kingfisher_gazebo.urdf.xacro` is what VRX spawns |
| `kingfisher_sim` | launch, `cmd_drive_translate` (normalized `cmd_drive` → thrust in N) and `sail_jog` (`sail/cmd_rate` → `sail/cmd_angle`) |

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
`cmd_drive_translate` in place of the MCU. The sail takes `sail/cmd_angle` (rad) through
`sail_relay` to its position controller, which turns it at a fixed speed and holds it, and
`sail/cmd_rate` (rad/s) through `sail_jog`, which turns it while the rate streams in. It also runs
on its own, once per Kingfisher, after any world that spawns one (`name:=<model name>`).

## Wind

The true wind is VRX's `USVWind`: uniform, with gusts, on `/vrx/debug/wind/{speed,direction}` (m/s; degrees
counter-clockwise from east, where the wind goes). On the lake it is set per launch and calm by default:

```
ros2 launch kingfisher_sim kingfisher_sim.launch.py wind_speed:=5 wind_direction:=0 wind_gust:=1
```

Other VRX worlds keep the wind in their SDF, and their `USVWind` gives the Kingfisher no hull windage.

`kingfisher_gazebo`'s `ApparentWind` system uses the apparent wind (true wind − the point's velocity) for:
- the sail: NACA 0018 lift and drag at the `sail_link` origin. Sim only: `sail/wrench` (`WrenchStamped`,
  in `sail_link`);
- the anemometer: `sensors/anemometer/wind` (`Vector3Stamped`, m/s, in `anemometer_link`: x forward,
  y starboard, z always 0), pointing where the wind goes, low-passed with τ = 1 s. The boat's
  anemometer driver publishes the same.

`robot.launch.py`'s `wind_bridge` brings both to ROS. `anemometer_relay` turns the bridged
`sensors/anemometer/twist` (sim only) into `sensors/anemometer/wind`, since the bridge has no
`Vector3Stamped`. The table, sail area and τ are properties in `kingfisher_gazebo_plugins.xacro`.

Spawn configs name it as `model_type: usv` + `urdf: package://kingfisher_gazebo/urdf/kingfisher_gazebo.urdf.xacro`
(see `kingfisher_sim/config/kingfisher_spawn.yaml`).

`config/joy_teleop.yaml` sends Newtons straight to `thrusters/*`, bypassing `cmd_drive_translate`.
It is for debugging only; normal teleop goes through `cmd_drive` or `cmd_vel`, as on the boat.
