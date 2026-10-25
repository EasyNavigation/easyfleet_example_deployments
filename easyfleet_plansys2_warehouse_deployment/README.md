# easyfleet_plansys2_warehouse_deployment

Four Kobukis in Warehouse A of [large_warehouse_world](https://github.com/juanscelyg/large_warehouse_world), driven by [PlanSys2](https://plansys2.github.io/). Two scouts locate packages in the rack bays, and two carriers take them to the docks. The mission controller builds the PDDL problem from the capabilities the robots announce, and PlanSys2 plans each goal with Kairos. Each action of the plan runs a behavior tree, and the leaves of that tree call the robot's EasyFleet capabilities through the [easyfleet_bt_nodes](https://github.com/EasyNavigation/EasyFleet/tree/rolling/easyfleet_bt_nodes) plugins. Navigation is real EasyNav; perception and manipulation are mocks (a Kobuki has no camera or arm).

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Robot | Role | Home (map) | Capabilities |
|---|---|---|---|
| `robot_1` | scout | `home_1` (0, 0) | `navigation` (EasyNav), `perception` (fake, detects `package`) |
| `robot_2` | scout | `home_2` (2, 1) | `navigation` (EasyNav), `perception` (fake, detects `package`) |
| `robot_3` | carrier | `home_3` (4, 0) | `navigation` (EasyNav), `manipulation` (fake, 8 s per task) |
| `robot_4` | carrier | `home_4` (6, 1) | `navigation` (EasyNav), `manipulation` (fake, 8 s per task) |

Each robot (`launch/robot_launch.yaml`, under its namespace) runs:

- `system_main`: EasyNav on easyfleet_easynav_deployment's `maps/warehouse_a.yaml` and `maps/warehouse_a_routes.yaml` (`config/easynav_system.params.yaml`). Its initial pose comes from `config/initial_poses.yaml`.
- `navigation`: `easynav_navigation_capability_node`, with the waypoints of `config/navigation_params.yaml`.
- `robot_node`: from easyfleet_easynav_collaboration_deployment, hosting the capability of its role (`config/scout.yaml` or `config/carrier.yaml`).

Central processes (`launch/warehouse_launch.yaml`):

- `navigation_manager_node`: the fleet's map and routes, plus the TF relay for the central RViz.
- `central_rviz`: `rviz/warehouse.rviz`, with each robot's `base_link` and planned path.
- PlanSys2 (`plansys2_bringup` monolithic, Kairos, `config/plansys2_params.yaml`).
- One `bt_action_node` per action a robot may run at once: `move_1`..`move_4`, `inspect_1`..`inspect_2`, `pick_1`..`pick_2` and `drop_1`..`drop_2`.
- `warehouse_controller`: started 10 s after the rest.

## The PDDL model

`pddl/warehouse.pddl` has four durative actions. Each one runs a tree from `behavior_trees/`:

| Action | Behavior tree | Capability called |
|---|---|---|
| `move ?r ?from ?to` | `CallNavigation` to the waypoint `?to` | `/<robot>/navigation` |
| `inspect ?r ?p ?l` | `CallPerception` until a `package` is detected (20 s limit) | `/<robot>/perception` |
| `pick ?r ?p ?l` | `CallManipulation`, named task `pick` | `/<robot>/manipulation` |
| `drop ?r ?p ?l` | `CallManipulation`, named task `place` | `/<robot>/manipulation` |

The locations are the waypoints of the navigation capability, with the same names: four homes, the bays `bay_1`..`bay_4` in the aisles of the bottom rack block, and the docks `dock_2`..`dock_5` at the end of the dock spurs. Only one robot can be at a location (`free`). A carrier can only pick a package that a scout has inspected (`located`). The duration of `move` comes from the `distance` between the two locations, which the controller computes along the grid of aisles and corridors.

## The mission

`warehouse_controller` (`src/warehouse_controller.cpp`, `config/controller.yaml`) works in four steps:

1. It waits until every robot announces `navigation` plus `perception` or `manipulation` (`FleetSession`).
2. It writes the problem from what it found: `can_perceive` for a robot with perception, `can_manipulate` for a robot with manipulation. A robot that never announces navigation is left out of the problem.
3. It adds the packages, the docks and the distances.
4. It reaches each goal of `goals` in turn: plan, print the plan, run it and report each action as it starts and ends.

The goals are:

1. `pkg_1` (bay_2) to `dock_3` and `pkg_2` (bay_3) to `dock_4`.
2. `pkg_3` (bay_1) to `dock_2` and `pkg_4` (bay_4) to `dock_5`.
3. Every robot back home.

## Running it

```bash
# Terminal 1: Gazebo with the four Kobukis and the Zenoh router
ros2 launch easyfleet_plansys2_warehouse_deployment simulation_launch.yaml

# Terminal 2: the fleet, PlanSys2 and the mission
ros2 launch easyfleet_plansys2_warehouse_deployment warehouse_launch.yaml
```

Launch arguments:

- `simulation_launch.yaml`:
  - `gui:=false` runs Gazebo headless.
  - `world:=` chooses the world. By default it is `large_warehouse_world_a.sdf`, which only furnishes Warehouse A, so that the four lidars keep the simulation in real time.
- `warehouse_launch.yaml`:
  - `mission:=false` starts everything but the controller; the goals can then be set from the PlanSys2 terminal (`ros2 run plansys2_terminal plansys2_terminal`).
  - `central_rviz:=false` skips the central RViz.

The first goal takes about 6 minutes of simulated time.
