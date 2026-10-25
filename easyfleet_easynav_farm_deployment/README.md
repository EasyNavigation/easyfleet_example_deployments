# easyfleet_easynav_farm_deployment

One Summit XL navigating outdoors on a farm (the `granja_leon` world). Its EasyFleet `navigation` capability is backed by EasyNav with a different stack from the indoor examples:

- **Localization:** GPS + IMU + wheel odometry fusion (`easynav_fusion_localizer/FusionLocalizer`), with no laser or AMCL.
- **Map:** a **NavMap** (`easynav_navmap_maps_manager`), with preferred routes overlaid by `easynav_navmap_routes_maps_manager`.
- **Planning and control:** an A* planner on the NavMap (`easynav_navmap_planner`) and the regulated pure-pursuit controller.

It shows that the capability, the BT and the waypoint interface from [easyfleet_easynav_deployment](../easyfleet_easynav_deployment) work unchanged on a different robot, environment and EasyNav configuration. This package has only launch and config files, and no code of its own.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Process | Namespace | What it is |
|---|---|---|
| Gazebo + Summit XL | none | `easynav_playground_summit_worlds` from [easynav_playgrounds](https://github.com/EasyNavigation/easynav_playgrounds), loaded with the `granja_leon` world. The robot is not namespaced: it publishes `/robotnik_base_control/odom`, `/gps/fix`, `/imu/data`, `/tf` and listens on `/cmd_vel`. |
| `system_main` | `/robot_1` | EasyNav: `FusionLocalizer`, NavMap and NavMap routes maps managers, A* planner, RPP controller. Its `cmd_vel` is remapped to `/cmd_vel`. |
| `navigation` | `/robot_1` | `easynav_navigation_capability_node` from `easyfleet_easynav_deployment`, with the waypoints `north_point` (0, 10), `east_point` (10, 0) and `home` (0, 0) in `map`. |
| `rviz2` | `/robot_1` | `rviz/robot_1.rviz`: NavMap, filtered odometry, planned path. |
| `rmw_zenohd` | none | Zenoh router, started by `simulation_launch.yaml`. |

The `navigation` capability is `/robot_1/navigation`. There is no mission node and no `navigation_manager_node` in this example, so you send the goals yourself.

## Requirements

`granja_leon`, the world, its models and `maps/granja_leon.navmap`, is a separate package that must be in your workspace. It is **not publicly released yet**, so `easyfleet_example_deployments.repos` does not include it. The fusion localizer also needs `robot_localization` (rolling-devel), which that `.repos` file does include.

## How to run

Build the workspace (with `granja_leon` in it) as described in the [top-level README](../README.md). Then:

```bash
# Terminal 1: Zenoh router + Gazebo with granja_leon + Summit XL
ros2 launch easyfleet_easynav_farm_deployment simulation_launch.yaml        # gui:=false for headless

# Terminal 2: EasyNav + navigation capability + RViz (rviz:=false to skip RViz)
ros2 launch easyfleet_easynav_farm_deployment robot_1_launch.yaml

# Terminal 3: send a goal to a named waypoint
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 action send_goal --feedback /robot_1/navigation \
  easyfleet_interfaces/action/Navigation "{parameters_json: '{\"goal_id\": \"north_point\"}'}"
```

The capability ignores `target_pose` and navigates to the waypoint named in `goal_id`.

## What you should see

- **Gazebo:** the farm and the Summit XL, which drives once a goal is sent.
- **RViz:** the NavMap, the filtered odometry from the fusion localizer and the planned A* path.
- **Terminal 3:** navigation feedback and a result with `error_code` 0 when the robot gets within `position_tolerance` (0.75 m) of the waypoint.

## Key files

| File | Purpose |
|---|---|
| `launch/simulation_launch.yaml` | Zenoh router, `GZ_SIM_RESOURCE_PATH` for the `granja_leon` models, and the Summit XL in the `granja_leon` world (`gui`). |
| `launch/robot_1_launch.yaml` | `system_main`, the navigation capability and RViz under `/robot_1` (`rviz`). |
| `config/easynav_system.params.yaml` | EasyNav stack. The GPS origin (42.630258, -5.551252), the local and global filters, the NavMap from `granja_leon`, A* and RPP. |
| `config/navigation_params.yaml` | Named waypoints. |
| `config/navigation.json` | Capability description, published on `/capabilities`. |
| `rviz/robot_1.rviz` | NavMap view (needs `navmap_rviz_plugin`). |

## Notes and troubleshooting

- Only the EasyFleet and EasyNav nodes are under `robot_1`. The simulated robot is not namespaced, and `tf_prefix` is not set, so EasyNav uses plain `map`, `odom` and `base_link`. The fixed frame in `rviz/robot_1.rviz` is `robot_1/map`. If RViz shows nothing, set Fixed Frame to `map`.
- In some environments (for example RoboStack/pixi rolling), EasyNav binaries currently fail with `undefined symbol: PyObject_GetAttrString`. This is an EasyNav linking issue. Workaround: `export LD_PRELOAD=$CONDA_PREFIX/lib/libpython3.14.so` (adjust the path to your Python).
