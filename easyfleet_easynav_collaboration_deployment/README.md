# easyfleet_easynav_collaboration_deployment

Two Kobukis in Gazebo, both navigating with real EasyNav on one shared map that a central `navigation_manager_node` publishes. `robot_1` also has a fake perception capability, and `robot_2` a fake manipulation capability. A five-phase mission coordinates them. This is the multi-robot version of [easyfleet_easynav_deployment](../easyfleet_easynav_deployment), written with the **low-level API**: `robot_node.cpp` configures and activates the capabilities by hand, and the mission uses `discover_capabilities()`, `CapabilityClient` and `run_capability()` directly. [easyfleet_easynav_collaboration_simple_api_deployment](../easyfleet_easynav_collaboration_simple_api_deployment) runs the same scenario with the simple `Deployment`/`SimpleController` API. The zenoh and fastdds variants add network isolation for each robot.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Robot | Namespace | Spawn | Capabilities |
|---|---|---|---|
| `robot_1` | `/robot_1` | (0, 0) | `navigation` (EasyNav), `perception` (fake, `gato` detections) |
| `robot_2` | `/robot_2` | (2, 1) | `navigation` (EasyNav), `manipulation` (fake, 8 s execution) |

Each robot runs these processes, all under its namespace:

- `system_main`: EasyNav with costmap AMCL, costmap planner and RPP. It uses `tf_prefix=robot_N`.
- `navigation`: `easynav_navigation_capability_node`, from `easyfleet_easynav_deployment`.
- `robot_node`: hosts the fake capability.
- Its own RViz, off by default.

Each robot has its own TF tree on `/robot_N/tf`.

Central, unnamespaced processes:

- `navigation_manager_node`:
  - Publishes the `home2` map on `/global_map` and the routes on `/global_routes`.
  - Relays `odom`, `base_footprint` and `base_link` from each robot's tree to `/tf` at 10 Hz (`robots.relay_tf*`), for the central RViz.
- `central_rviz`: `easyfleet_easynav_deployment/rviz/navigation_manager.rviz`, which shows the global map, the routes, each robot's `base_link` axes and each robot's planned path. It has no RobotModel display.
- `easynav_collaboration_mission_node`: started 5 s after the rest.
- `rmw_zenohd`: the Zenoh router, from `simulation_launch.yaml`.

## How to run

Build the workspace as described in the [top-level README](../README.md). Then:

```bash
# Terminal 1: Zenoh router + Gazebo (small house) + both Kobukis
ros2 launch easyfleet_easynav_collaboration_deployment simulation_launch.yaml        # gui:=false for headless

# Terminal 2: both robots + navigation manager + central RViz + mission
ros2 launch easyfleet_easynav_collaboration_deployment easynav_collaboration_launch.yaml
```

Add `robot_rviz:=true` to also open each robot's RViz, which shows its costmap, scan, particles, path and the mission status text. One robot can also be started on its own, for example `ros2 launch easyfleet_easynav_collaboration_deployment robot_1_launch.yaml`. In that case its RViz is on by default (`rviz:=false` turns it off).

## What you should see

- **Gazebo:** both Kobukis moving in the small house.
- **Central RViz:** the global map, the routes, both robots' `base_link` axes moving and each robot's current path.
- **Mission output.** Goals run until they finish, with a 180 s safety cap:
  0. **Phase 0:** discovery of the four capabilities.
  1. **Phase 1:** `robot_1` goes to `kitchen` with perception running, while `robot_2` goes to `dock`.
  2. **Phase 2:** `robot_1` moves to `kitchen_standby` to make room, while `robot_2` goes to `kitchen`.
  3. **Phase 3:** `robot_1`'s perception stops, and `robot_2` runs manipulation for up to 10 s.
  4. **Phase 4:** both robots head to `dock`. 10 s in, `robot_1` is stopped and sent to `charging_station` instead.
  5. **Phase 5:** both robots return home (`robot_1_home`, `robot_2_home`).
- **Per-robot RViz** (`robot_rviz:=true`): the mission also publishes a status line above each robot on `/mission_status_markers`.

## Warehouse A

The same two robots and mission in Warehouse A of [large_warehouse_world](https://github.com/juanscelyg/large_warehouse_world), on easyfleet_easynav_deployment's Warehouse A map and routes (see its README):

```bash
ros2 launch easyfleet_easynav_collaboration_deployment warehouse_simulation_launch.yaml
ros2 launch easyfleet_easynav_collaboration_deployment warehouse_launch.yaml
```

- `robot_1` spawns at (0, 0) and `robot_2` at (2, 1), in the central plaza. Of the mission's waypoints (`config/warehouse/navigation_params.yaml`), `dock`, `kitchen` and `kitchen_standby` are on the corridor between the two rack blocks, and `charging_station` and the homes in the plaza.
- Each Kobuki carries a cylinder at the height of its lidar (`lidar_target`), so that the other robot's lidar sees it; its lidar reaches 30 m.
- `robot_rviz:=true` opens each robot's RViz; `mission:=false` starts everything but the mission.
- With both robots the simulation keeps real time only in `large_warehouse_world_a.sdf` (the default): every lidar renders the whole world.

## Key files

| File | Purpose |
|---|---|
| `launch/simulation_launch.yaml` | Zenoh router, world and two namespaced Kobukis (`gui` argument). |
| `launch/easynav_collaboration_launch.yaml` | Both robots + `navigation_manager_node` + central RViz + mission (`robot_rviz` argument). |
| `launch/robot_N_launch.yaml` | One robot's stack under `/robot_N` (`rviz` argument). |
| `config/navigation_manager.params.yaml` | Global map and routes, and the TF relay settings. |
| `config/navigation_params.yaml`, `navigation.json` | Waypoints shared by both robots, and the navigation capability description. |
| `config/robot_N/easynav_system.costmap_rpp.params.yaml` | EasyNav parameters. The only difference between the robots is the AMCL initial pose. |
| `config/robot_N/robot_params.yaml` | Fake capabilities that `robot_node` hosts. |
| `rviz/robot_N.rviz` | Per-robot views, with topics already resolved to `/robot_N/...`. |
| `src/robot_node.cpp` | Hand-written lifecycle management of the fake capabilities. |
| `src/main_easynav_collaboration.cpp` | The five-phase mission (low-level API, explicit status markers). |

## Troubleshooting

- In some environments (for example RoboStack/pixi rolling), EasyNav binaries currently fail with `undefined symbol: PyObject_GetAttrString`. This is an EasyNav linking issue. Workaround: `export LD_PRELOAD=$CONDA_PREFIX/lib/libpython3.14.so` (adjust the path to your Python).
- If the mission finds no capabilities, the robots took longer than 5 s to activate. Relaunch, or start the mission by hand: `RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run easyfleet_easynav_collaboration_deployment easynav_collaboration_mission_node --ros-args -p use_sim_time:=true`.
- The central RViz only receives the three relayed frames of each robot. For a robot's full tree, use its own RViz (`robot_rviz:=true`).
