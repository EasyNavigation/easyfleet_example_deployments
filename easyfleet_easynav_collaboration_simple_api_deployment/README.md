# easyfleet_easynav_collaboration_simple_api_deployment

The two-robot EasyNav scenario of [easyfleet_easynav_collaboration_deployment](../easyfleet_easynav_collaboration_deployment): same robots, world, waypoints and five-phase mission. This version is written with the **simple API**, which is the one to copy for new deployments:

- **Robot side:** `robot_node.cpp` is about ten lines built on `easyfleet::Deployment`. It loads the fake capabilities as pluginlib plugins, named in the `capabilities` parameter, and takes care of their lifecycle.
- **Mission side:** the mission uses `easyfleet::SimpleController` and `RobotHandle` (`has_capability()`, `run_capability()`, ...). Status markers in RViz are published automatically for each goal.

There is also a Python version: [easyfleet_easynav_collaboration_simple_api_deployment_py](../easyfleet_easynav_collaboration_simple_api_deployment_py). The [zenoh](../easyfleet_easynav_collaboration_zenoh_deployment) and [fastdds](../easyfleet_easynav_collaboration_fastdds_deployment) variants are built on this package's code.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Robot | Namespace | Spawn | Capabilities |
|---|---|---|---|
| `robot_1` | `/robot_1` | (0, 0) | `navigation` (EasyNav), `perception` (fake, `gato` detections) |
| `robot_2` | `/robot_2` | (2, 1) | `navigation` (EasyNav), `manipulation` (fake, 8 s execution) |

Each robot runs these processes, all under its namespace:

- `system_main`: EasyNav with costmap AMCL, costmap planner and RPP. It uses `tf_prefix=robot_N`.
- `navigation`: `easynav_navigation_capability_node`, from `easyfleet_easynav_deployment`.
- `robot_node`: the `Deployment` that hosts the fake capability.
- Its own RViz, off by default.

Each robot has its own TF tree on `/robot_N/tf`.

Central, unnamespaced processes:

- `navigation_manager_node`:
  - Publishes `/global_map` and `/global_routes`.
  - Relays `odom`, `base_footprint` and `base_link` of each robot to `/tf` at 10 Hz.
- The central RViz: `navigation_manager.rviz` from `easyfleet_easynav_deployment`, showing the global map, the routes, each robot's `base_link` axes and each robot's path.
- `easynav_collaboration_mission_node`: started 5 s after the rest.
- `rmw_zenohd`: the Zenoh router, from `simulation_launch.yaml`.

## How to run

Build the workspace as described in the [top-level README](../README.md). Then:

```bash
# Terminal 1: Zenoh router + Gazebo (small house) + both Kobukis
ros2 launch easyfleet_easynav_collaboration_simple_api_deployment simulation_launch.yaml        # gui:=false for headless

# Terminal 2: both robots + navigation manager + central RViz + mission
ros2 launch easyfleet_easynav_collaboration_simple_api_deployment easynav_collaboration_launch.yaml
```

`robot_rviz:=true` also opens each robot's RViz. `robot_N_launch.yaml` starts a single robot, with its RViz on by default (`rviz:=false`).

## What you should see

The output is the same as in [easyfleet_easynav_collaboration_deployment](../easyfleet_easynav_collaboration_deployment#what-you-should-see):

- **Gazebo:** both Kobukis moving.
- **Central RViz:** the global map, the routes, both `base_link` axes and the planned paths.
- **Mission output:**
  - **Phase 0:** discovery, with each robot's capabilities printed by `RobotHandle::print_capabilities()`.
  - **Phases 1-5:** `kitchen`/`dock` → `kitchen_standby`/`kitchen` → manipulation at `kitchen` → `dock` with `robot_1` redirected to `charging_station` → back home. Goals have a 180 s safety cap.
- If a required capability is missing, the mission stops at the start with a red error. Both robots need `navigation`, `robot_1` needs `perception` and `robot_2` needs `manipulation`.

## Key files

| File | Purpose |
|---|---|
| `launch/simulation_launch.yaml` | Zenoh router, world and two namespaced Kobukis. |
| `launch/easynav_collaboration_launch.yaml` | Both robots + `navigation_manager_node` + central RViz + mission (`robot_rviz`). |
| `launch/robot_N_launch.yaml` | One robot's stack under `/robot_N` (`rviz`). |
| `config/robot_N/robot_params.yaml` | `capabilities` (pluginlib lookup names) and `config_subdir` for the `Deployment`. |
| `config/robot_N/*` | EasyNav parameters, fake capability parameters and JSON descriptions. |
| `config/navigation_params.yaml`, `navigation.json` | Shared waypoints and the navigation capability description. |
| `config/navigation_manager.params.yaml` | Global map and routes, and the TF relay. |
| `easyfleet_easynav_collaboration_simple_api_deployment_plugins.xml` | Registers the `perception` and `manipulation` capability factories. |
| `src/robot_node.cpp` | `Deployment`: `add_capabilities_from_parameters()`, then `start()`, `run()` and `shutdown()`. |
| `src/main_easynav_collaboration.cpp` | The five-phase mission on `SimpleController`/`RobotHandle`. |

## Troubleshooting

- In some environments (for example RoboStack/pixi rolling), EasyNav binaries currently fail with `undefined symbol: PyObject_GetAttrString`. This is an EasyNav linking issue. Workaround: `export LD_PRELOAD=$CONDA_PREFIX/lib/libpython3.14.so` (adjust the path to your Python).
- Status text markers (`/mission_status_markers`) are shown only in the per-robot RViz (`robot_rviz:=true`).
