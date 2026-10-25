# easyfleet_easynav_deployment

One Kobuki in Gazebo whose `navigation` capability is backed by real [EasyNavigation](https://easynavigation.github.io/). It uses a costmap AMCL localizer, a costmap planner and a regulated pure-pursuit controller. The robot's own navigation action server runs a BehaviorTree.CPP tree (`StartOff -> Navigate -> Finish`), and the `Navigate` node drives EasyNav. The fake examples simulate navigation; this one drives a simulated robot to named waypoints. The package also provides the parts the multi-robot EasyNav examples reuse: `easynav_navigation_capability_node`, the BT plugins, the `home2` map, the routes and the central `navigation_manager.rviz`.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

By default everything is unnamespaced, since there is a single robot. `robot_namespace:=<ns>` pushes the robot nodes under a namespace.

| Process | What it is |
|---|---|
| Gazebo + Kobuki | Small house world from [easynav_playgrounds](https://github.com/EasyNavigation/easynav_playgrounds) (`easynav_playground_kobuki_worlds`), with one robot at (0, 0). |
| `system_main` | EasyNav: costmap maps manager plus routes maps manager, AMCL localizer, costmap planner, RPP controller. |
| `navigation` (`easynav_navigation_capability_node`) | The EasyFleet `navigation` capability. Waypoints: `dock`, `kitchen`, `charging_station`. |
| `rviz2` | Robot view (`rviz/easynav_costmap.rviz`): costmap, scan, particles, path. |
| `navigation_manager_node` | Publishes the map on `/global_map` and the routes on `/global_routes` (transient local). `system_main` takes them through remaps. |
| `easynav_mission_node` | Mission control, started 3 s after the rest. |
| `rmw_zenohd` | Zenoh router, started by `simulation_launch.yaml`. |

## How to run

Build the workspace as described in the [top-level README](../README.md). Then:

```bash
# Terminal 1: Zenoh router + Gazebo + Kobuki
ros2 launch easyfleet_easynav_deployment simulation_launch.yaml        # gui:=false for headless

# Terminal 2: EasyNav + capability + RViz + navigation manager + mission
ros2 launch easyfleet_easynav_deployment easynav_gazebo_launch.yaml
```

`easynav_robot_gazebo_launch.yaml` starts only the robot side (`system_main`, the capability and RViz). Use it if you want to send goals yourself.

## What you should see

- **Gazebo:** the small house with the Kobuki.
- **RViz:** the costmap, the laser scan, the AMCL particles and the planned path while the robot moves.
- **Mission output:**
  1. **Phase 1:** discovery and the description of the capability.
  2. **Phase 2:** navigation to `dock`. `StartOff` and `Finish` print a message before and after the trip. The phase is capped at 10 s, so the goal may be stopped before the robot arrives.
  3. **Phase 3:** a goal to `dock`, then 3 s later a goal to `kitchen`. The first goal is aborted at the ROS level, and EasyNav is redirected to the new target without stopping.

To send a goal by hand:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 action send_goal /navigation easyfleet_interfaces/action/Navigation \
  "{parameters_json: '{\"goal_id\": \"kitchen\"}'}"
```

## The EasyNav-backed navigation capability

This backend ignores `target_pose`. It reads a waypoint id from `parameters_json` (`{"goal_id": "<id>"}`) and looks it up in `navigation.waypoints` (`config/easynav_robot/navigation_params.yaml`). The action runs the BT given in `behavior_tree_xml` with the plugins listed in `bt_plugins`:

- `Navigate`, from `easyfleet_easynav_navigation`.
- `StartOff` and `Finish`, from this package.

The capability keeps a persistent EasyNav goal client, so a new goal redirects EasyNav instead of stopping it first.

## tf_prefix and namespaces

Each robot has its own TF tree under its namespace (`/robot_N/tf`, remapped from `/tf`). The Kobuki publishes frames prefixed with that namespace (`robot_N/base_link`, ...). So when `robot_namespace` is set, `system_main` gets `tf_prefix:=<robot_namespace>` and uses the same frame names. With the default empty namespace, frames are plain `map`/`odom`/`base_link`.

## Warehouse A

The same robot in Warehouse A of [large_warehouse_world](https://github.com/juanscelyg/large_warehouse_world), a 200 x 200 m warehouse with rack aisles:

```bash
# Terminal 1: Gazebo with Warehouse A and a Kobuki at (0, 0) of the map
ros2 launch easyfleet_easynav_deployment warehouse_simulation_launch.yaml

# Terminal 2: EasyNav and its RViz, the navigation capability and the navigation manager
ros2 launch easyfleet_easynav_deployment warehouse_launch.yaml
```

Goals come from RViz's 2D Goal Pose. `mission:=true` also runs `easynav_mission_node`, whose 10 s per goal suit the small house rather than this warehouse.

- **World**: `large_warehouse_world_a.sdf` by default (`world:=` for the whole one), with only Warehouse A furnished. The Kobuki's lidar reaches 30 m (`lidar_range`): at 12 m it sees almost nothing in the open areas.
- **Map** (`maps/warehouse_a.yaml`, 0.2 m, origin at the center of Warehouse A): what the lidar sees at its height (rack legs, the packages under the racks, walls). `scripts/sdf_to_costmap.py` generates it from the world; `maps/warehouse_a_collision.yaml` is the same band from the collision geometry (solid racks).
- **Routes** (`maps/warehouse_a_routes.yaml`): one per rack aisle, two along the central aisle, the edges of the rack area, the cross corridors and spurs to the docks. `scripts/aisle_routes.py` generates them from the maps.
- **EasyNav** (`config/warehouse/`): the costmap/RPP stack of this package on those maps; the map server's update rate is 2 Hz, as 10 Hz is not kept with 1010 x 1010 cells.

## Key files

| File | Purpose |
|---|---|
| `launch/simulation_launch.yaml` | Zenoh router, Gazebo and the Kobuki (`gui` argument). |
| `launch/easynav_gazebo_launch.yaml` | Robot side + `navigation_manager_node` + mission (`robot_namespace` argument). |
| `launch/easynav_robot_gazebo_launch.yaml` | `system_main`, the capability and RViz, with the TF and global map remaps. |
| `config/easynav_robot/easynav_system.costmap_rpp.params.yaml` | EasyNav stack parameters. |
| `config/easynav_robot/navigation_params.yaml` | Named waypoints and BT tick rate. |
| `config/easynav_robot/navigation_manager.params.yaml` | Map and routes that `navigation_manager_node` publishes. |
| `config/easynav_robot/navigation_gazebo.json` | Capability description. |
| `behavior_trees/navigate.xml` | The `StartOff -> Navigate -> Finish` tree. |
| `maps/` | `home1`/`home2` occupancy maps, `routes_1.yaml`. |
| `rviz/easynav_costmap.rviz` | Single-robot view. |
| `rviz/navigation_manager.rviz` | Fleet view, used by the multi-robot examples. |
| `src/<package>/easynav_navigation_capability.cpp` | Capability implementation. |
| `src/<package>/start_off.cpp`, `finish.cpp` | Demo BT nodes. |
| `src/main_easynav.cpp` | The mission. |

## Troubleshooting

- In some environments (for example RoboStack/pixi rolling), EasyNav binaries currently fail with `undefined symbol: PyObject_GetAttrString`. This is an EasyNav linking issue. Workaround: `export LD_PRELOAD=$CONDA_PREFIX/lib/libpython3.14.so` (adjust the path to your Python).
- Mission phases are capped at 10 s, which is shorter than a full trip across the house. The multi-robot examples let goals run until they finish.
