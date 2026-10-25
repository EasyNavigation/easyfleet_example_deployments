# easyfleet_easynav_collaboration_simple_api_deployment_py

The Python version of [easyfleet_easynav_collaboration_simple_api_deployment](../easyfleet_easynav_collaboration_simple_api_deployment): same two robots, world, waypoints, central view and five-phase mission. The fake perception and manipulation capabilities and the mission are written in Python on `easyfleet_mission_manager_py`, with `Capability`/`ActionServerBase` for the capabilities and `RobotHandle`/`SimpleController` for the mission. Navigation is still real EasyNav, using the C++ `easynav_navigation_capability_node` from `easyfleet_easynav_deployment`. Use this package as a template if your mission or your capabilities are in Python. It is also an example of C++ and Python capabilities in the same fleet.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Robot | Namespace | Spawn | Capabilities |
|---|---|---|---|
| `robot_1` | `/robot_1` | (0, 0) | `navigation` (EasyNav, C++), `perception` (fake, Python) |
| `robot_2` | `/robot_2` | (2, 1) | `navigation` (EasyNav, C++), `manipulation` (fake, Python) |

Each robot runs these processes, all under its namespace:

- `system_main`: EasyNav, with `tf_prefix=robot_N` and its own TF tree on `/robot_N/tf`.
- `navigation`: the C++ capability node.
- One Python process for its fake capability: `perception_fake_capability_node` on `robot_1`, `manipulation_fake_capability_node` on `robot_2`.
- Its own RViz, off by default.

Central processes: `navigation_manager_node`, the central RViz (`navigation_manager.rviz`), the Python `easynav_collaboration_mission_node` (5 s delay) and the Zenoh router started by `simulation_launch.yaml`. `navigation_manager_node` publishes `/global_map` and `/global_routes`, and relays `odom`, `base_footprint` and `base_link` of each robot to `/tf` at 10 Hz.

This package has no `config/` or `rviz/` folder of its own. The launch files take the EasyNav, capability, navigation manager and RViz configuration from `easyfleet_easynav_collaboration_simple_api_deployment`, so that package must be built too.

## How to run

Build the workspace as described in the [top-level README](../README.md). Then:

```bash
# Terminal 1: Zenoh router + Gazebo (small house) + both Kobukis
ros2 launch easyfleet_easynav_collaboration_simple_api_deployment_py simulation_launch.yaml        # gui:=false for headless

# Terminal 2: both robots + navigation manager + central RViz + Python mission
ros2 launch easyfleet_easynav_collaboration_simple_api_deployment_py easynav_collaboration_launch.yaml
```

`robot_rviz:=true` also opens each robot's RViz. `robot_N_launch.yaml` starts one robot, with its RViz on by default (`rviz:=false`).

## What you should see

The same as the [C++ version](../easyfleet_easynav_collaboration_simple_api_deployment#what-you-should-see):

- **Gazebo:** both Kobukis moving.
- **Central RViz:** the global map, the routes, both `base_link` axes and the paths.
- **Mission output:**
  - **Phase 0:** discovery.
  - **Phases 1-5:** `kitchen`/`dock` → `kitchen_standby`/`kitchen` → manipulation → `dock` with `robot_1` redirected to `charging_station` → home. 180 s safety cap per goal.
- **Per-robot RViz:** status markers, published automatically by `RobotHandle.run_capability()`.

## Key files

| File | Purpose |
|---|---|
| `launch/simulation_launch.yaml` | Zenoh router, world and two namespaced Kobukis. |
| `launch/easynav_collaboration_launch.yaml` | Both robots + `navigation_manager_node` + central RViz + mission (`robot_rviz`). |
| `launch/robot_N_launch.yaml` | One robot: `system_main`, the C++ navigation capability and the Python fake capability (`rviz`). |
| `<package>/mission.py` | The five-phase mission on `SimpleController`/`RobotHandle`. |
| `<package>/perception_fake_capability.py` | Python fake perception (`gato` detections). |
| `<package>/manipulation_fake_capability.py` | Python fake manipulation. |
| `setup.py` | Console scripts: `easynav_collaboration_mission_node`, `perception_fake_capability_node`, `manipulation_fake_capability_node`. |

## Troubleshooting

- In some environments (for example RoboStack/pixi rolling), EasyNav binaries currently fail with `undefined symbol: PyObject_GetAttrString`. This is an EasyNav linking issue. Workaround: `export LD_PRELOAD=$CONDA_PREFIX/lib/libpython3.14.so` (adjust the path to your Python).
- Because there is one process per Python capability, a robot that needs more capabilities needs one more node in its `robot_N_launch.yaml` for each.
