# easyfleet_easynav_collaboration_zenoh_deployment

The [simple API two-robot scenario](../easyfleet_easynav_collaboration_simple_api_deployment) (same code, waypoints and mission) deployed as separate **Zenoh islands**:

- Each robot has its own `rmw_zenohd` router, and its nodes connect only to that router.
- Each robot router connects to a central router.
- An access-control list (ACL) on each robot router lets only a small set of topics cross to the central island. Internal topics such as odometry, scan and costmaps stay on the robot.

In the other EasyNav examples, every node shares one router, so every topic is visible everywhere. This example shows how to keep each robot's traffic local while the fleet still works, as you would with separate machines on a shared network. [The fastdds variant](../easyfleet_easynav_collaboration_fastdds_deployment) does the same with DDS domains and DDS Router.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Island | Router (config) | Endpoints | What runs there |
|---|---|---|---|
| central | `config/zenoh_router_central.json5` | `tcp/127.0.0.1:7447` | Gazebo world + `/clock` bridge, `navigation_manager_node`, central RViz, mission |
| `robot_1` | `config/robot_1/zenoh_router.json5` | unix socket `/tmp/easyfleet_robot_1_zenoh.sock`, `tcp/127.0.0.1:7448` | Kobuki 1, and under `/robot_1`: `system_main`, `navigation`, `robot_node` (`perception`), optional RViz |
| `robot_2` | `config/robot_2/zenoh_router.json5` | unix socket `/tmp/easyfleet_robot_2_zenoh.sock`, `tcp/127.0.0.1:7449` | Kobuki 2, and under `/robot_2`: `system_main`, `navigation`, `robot_node` (`manipulation`), optional RViz |

How the islands connect:

- Nodes connect to their island's router through a session config: `zenoh_session_central.json5` or `robot_N/zenoh_session.json5`.
- Each robot router connects to the central router. `robot_1` and `robot_2` never connect directly to each other.

The robot routers deny by default. Clients on the local unix socket get everything. Over the TCP link to the central router, only these topics cross, in both directions:

- `/capabilities`, `/capabilities_status`
- `/global_map`, `/global_routes`
- `/clock`, `/mission_status_markers`
- The robot's capability actions, `planner_node` and `easynav_control` topics.
- `/robot_N/tf` and `/robot_N/tf_static`, outbound only, so `navigation_manager_node` can relay `odom`, `base_footprint` and `base_link` to `/tf` for the central RViz.

Robots, spawn poses (0, 0) and (2, 1), and capabilities are the same as in the [simple API example](../easyfleet_easynav_collaboration_simple_api_deployment#what-runs-where).

## How to run

Build the workspace as described in the [top-level README](../README.md). Then:

```bash
# Terminal 1: the three Zenoh routers + Gazebo (small house) + both Kobukis, each in its island
ros2 launch easyfleet_easynav_collaboration_zenoh_deployment simulation_launch.yaml        # gui:=false for headless

# Terminal 2: robots (each with its router) + navigation manager + central RViz + mission
ros2 launch easyfleet_easynav_collaboration_zenoh_deployment easynav_collaboration_launch.yaml
```

`robot_rviz:=true` also opens each robot's RViz, inside that robot's island.

To inspect an island from another terminal, source its script (sourcing, not executing it, is required) and use the `ros2` CLI:

```bash
SCRIPTS=$(ros2 pkg prefix easyfleet_easynav_collaboration_zenoh_deployment)/share/easyfleet_easynav_collaboration_zenoh_deployment/scripts
source $SCRIPTS/set_robot_1.sh
ros2 topic list      # robot_1's full graph
source $SCRIPTS/set_central.sh
ros2 topic list      # only what the ACLs let through
```

The scripts set `RMW_IMPLEMENTATION` and `ZENOH_SESSION_CONFIG_URI`, and restart the `ros2` daemon. When several terminals are on different islands, use `--no-daemon`.

## What you should see

The same Gazebo scene, central RViz (global map, routes, both `base_link` axes, paths) and five-phase mission output as in the [simple API example](../easyfleet_easynav_collaboration_simple_api_deployment#what-you-should-see). The difference shows in `ros2 topic list`: from the central island you do not see `/robot_N/scan_raw`, `/robot_N/odom` and similar topics, but from a robot's island you do.

## Key files

| File | Purpose |
|---|---|
| `launch/simulation_launch.yaml` | Starts the three routers, then the world (central session) and each Kobuki (in its robot's session). |
| `launch/easynav_collaboration_launch.yaml` | Central router + both robots + central nodes, with the central session config. |
| `launch/robot_N_launch.yaml` | The robot's router + its stack, with its session config (`rviz` argument). |
| `config/zenoh_router_central.json5`, `zenoh_session_central.json5` | Central router and client sessions. |
| `config/robot_N/zenoh_router.json5` | Robot router: link to the central router, listen endpoints, and the ACL that defines what crosses. |
| `config/robot_N/zenoh_session.json5` | Client session over the robot's unix socket. |
| `config/robot_N/*`, `config/navigation*.{yaml,json}` | EasyNav, capability and navigation manager configuration, as in the simple API example. |
| `scripts/set_central.sh`, `set_robot_N.sh` | Source these to attach a shell to an island. |
| `src/robot_node.cpp`, `src/main_easynav_collaboration.cpp` | `Deployment` and `SimpleController` code, same as the simple API example. |

## Notes and troubleshooting

- Both `simulation_launch.yaml` and `easynav_collaboration_launch.yaml` start routers on the same endpoints. When both launch files are running, the routers from the second one find those endpoints already in use.
- The routers listen only on `127.0.0.1` and unix sockets, so everything runs on one machine. To spread islands across hosts, change the endpoints.
- In some environments (for example RoboStack/pixi rolling), EasyNav binaries currently fail with `undefined symbol: PyObject_GetAttrString`. This is an EasyNav linking issue. Workaround: `export LD_PRELOAD=$CONDA_PREFIX/lib/libpython3.14.so` (adjust the path to your Python).
