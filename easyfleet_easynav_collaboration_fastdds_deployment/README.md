# easyfleet_easynav_collaboration_fastdds_deployment

The [simple API two-robot scenario](../easyfleet_easynav_collaboration_simple_api_deployment) (same code, waypoints and mission) isolated per robot with **Fast DDS**, not Zenoh. Each robot runs in its own `ROS_DOMAIN_ID`, and the central part runs in domain 0. One eProsima **DDS Router** per robot bridges its domain with domain 0. An allowlist on each router decides which topics cross, so a robot's internal traffic never leaves its domain. It is the DDS version of the [zenoh variant](../easyfleet_easynav_collaboration_zenoh_deployment), and the only example that pins `rmw_fastrtps_cpp` instead of `rmw_zenoh_cpp`.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Domain | What runs there |
|---|---|
| 0 (central) | Gazebo world + `/clock` bridge, `navigation_manager_node`, central RViz, `easynav_collaboration_mission_node` |
| 1 (`robot_1`) | Kobuki 1, and under `/robot_1`: `system_main`, `navigation`, `robot_node` (`perception`), optional RViz |
| 2 (`robot_2`) | Kobuki 2, and under `/robot_2`: `system_main`, `navigation`, `robot_node` (`manipulation`), optional RViz |

`ddsrouter` processes: one for each robot, started by `robot_N_launch.yaml` with `config/robot_N/dds_router.yaml`. Each joins its robot's domain and domain 0. Its allowlist lets these topics cross:

- `capabilities`, `capabilities_status`
- `global_map`, `global_routes`
- `clock`, `mission_status_markers`
- The robot's capability actions, `planner_node` and `easynav_control`.
- `/robot_N/tf` and `/robot_N/tf_static`, routed only from the robot to the central domain, so `navigation_manager_node` can relay `odom`, `base_footprint` and `base_link` to `/tf` for the central RViz.

Transient-local topics such as `/global_map` and `/capabilities` get explicit QoS overrides so that late joiners still receive them.

Robots, spawn poses (0, 0) and (2, 1), and capabilities are the same as in the [simple API example](../easyfleet_easynav_collaboration_simple_api_deployment#what-runs-where).

## How to run

DDS Router is not a ROS package. Build it once with the script in this package. It installs to `$HOME/.local/dds_router`, which is the path the launch files use. Run it from a shell where your ROS environment is sourced, so it reuses the installed Fast DDS:

```bash
$(ros2 pkg prefix easyfleet_easynav_collaboration_fastdds_deployment)/share/easyfleet_easynav_collaboration_fastdds_deployment/scripts/build_dds_router.sh
```

If you pass a different install prefix as the first argument, edit the `ddsrouter` path in `robot_N_launch.yaml` to match.

Build the workspace as described in the [top-level README](../README.md). Then:

```bash
# Terminal 1: Gazebo world in domain 0, Kobuki N in domain N
ros2 launch easyfleet_easynav_collaboration_fastdds_deployment simulation_launch.yaml        # gui:=false for headless

# Terminal 2: DDS Routers + both robots + navigation manager + central RViz + mission
ros2 launch easyfleet_easynav_collaboration_fastdds_deployment easynav_collaboration_launch.yaml
```

`robot_rviz:=true` also opens each robot's RViz, in that robot's domain.

To inspect a domain from another terminal, source the script for that domain:

```bash
SCRIPTS=$(ros2 pkg prefix easyfleet_easynav_collaboration_fastdds_deployment)/share/easyfleet_easynav_collaboration_fastdds_deployment/scripts
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source $SCRIPTS/set_robot_1.sh    # ROS_DOMAIN_ID=1; also set_robot_2.sh (2), set_central.sh (0)
ros2 topic list
```

The scripts set `ROS_DOMAIN_ID` and restart the `ros2` daemon. They do not set the RMW. When several terminals use different domains, use `--no-daemon`.

## What you should see

The same Gazebo scene, central RViz (global map, routes, both `base_link` axes, paths) and five-phase mission output as in the [simple API example](../easyfleet_easynav_collaboration_simple_api_deployment#what-you-should-see).

The mission starts **about 40 s** after the robots. That margin is for EasyNav's first localization and costmap cycle, which can take more than 20 s on a loaded machine. It is not related to DDS Router.

From domain 0, `ros2 topic list` shows only the bridged topics.

## Key files

| File | Purpose |
|---|---|
| `launch/simulation_launch.yaml` | World in domain 0 and the Kobukis in domains 1 and 2 (`gui`). Pins `rmw_fastrtps_cpp`. |
| `launch/easynav_collaboration_launch.yaml` | Both robots + central nodes in domain 0, with the mission delayed 40 s (`robot_rviz`). |
| `launch/robot_N_launch.yaml` | The robot's `ddsrouter` + its stack in domain N (`rviz`). |
| `config/robot_N/dds_router.yaml` | Bridge participants (domain N and domain 0), allowlist, QoS overrides and TF routes. |
| `config/robot_N/*`, `config/navigation*.{yaml,json}` | EasyNav, capability and navigation manager configuration, as in the simple API example. |
| `scripts/build_dds_router.sh` | Builds DDS Router from source into `$HOME/.local/dds_router`. |
| `scripts/set_central.sh`, `set_robot_N.sh` | Source these to set a shell's `ROS_DOMAIN_ID`. |
| `src/robot_node.cpp`, `src/main_easynav_collaboration.cpp` | `Deployment` and `SimpleController` code, same as the simple API example. |

## Notes and troubleshooting

- Each robot's `robot_N_launch.yaml` expects `ddsrouter` at `$HOME/.local/dds_router/bin/ddsrouter`. If that file is missing, the bridge does not start, the robot's capabilities never reach domain 0, and the mission discovers nothing.
- Domains 0, 1 and 2 are fixed in the launch files and the `dds_router.yaml` files. If other ROS 2 systems on your network use these domains, their traffic mixes with this example.
- In some environments (for example RoboStack/pixi rolling), EasyNav binaries currently fail with `undefined symbol: PyObject_GetAttrString`. This is an EasyNav linking issue. Workaround: `export LD_PRELOAD=$CONDA_PREFIX/lib/libpython3.14.so` (adjust the path to your Python).
