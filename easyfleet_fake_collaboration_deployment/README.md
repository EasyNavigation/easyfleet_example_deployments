# easyfleet_fake_collaboration_deployment

Three robots with fake capabilities and one mission node that coordinates them. This is [easyfleet_fake_alone_deployment](../easyfleet_fake_alone_deployment) with more than one robot. Each robot carries a different set of capabilities under its own namespace, and mission control finds each capability by robot and by type, then runs some of them in parallel across robots and some in sequence on one robot. There is no simulator and nothing moves, so it starts in seconds. For the same kind of coordination with real navigation in Gazebo, see [easyfleet_easynav_collaboration_deployment](../easyfleet_easynav_collaboration_deployment).

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Robot | Namespace | Capabilities |
|---|---|---|
| `robot_1` | `/robot_1` | `navigation`, `perception` |
| `robot_2` | `/robot_2` | `navigation`, `perception` |
| `robot_3` | `/robot_3` | `navigation`, `manipulation` |

Each robot runs as one `robot_node` process that hosts its capabilities. The capabilities to host come from `config/robot_N/robot_params.yaml`. Central processes:

- `collaboration_mission_node`: mission control, started 3 s after the robots.
- `rmw_zenohd`: the Zenoh router, started by `collaboration_launch.yaml`.

The capabilities are the same mocks as in the alone example:

- `navigation`: simulates 5 s of travel.
- `perception`: streams `gato` detections at 20 Hz until it is canceled.
- `manipulation`: simulates a 3 s trajectory.

## How to run

Build the workspace as described in the [top-level README](../README.md), then run:

```bash
source install/setup.bash
ros2 launch easyfleet_fake_collaboration_deployment collaboration_launch.yaml
```

To bring the robots up one by one, for example in separate terminals or on separate machines that share one router, start the router first:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd                                              # Terminal 1
ros2 launch easyfleet_fake_collaboration_deployment robot_1_launch.yaml        # Terminal 2
ros2 launch easyfleet_fake_collaboration_deployment robot_2_launch.yaml        # Terminal 3
ros2 launch easyfleet_fake_collaboration_deployment robot_3_launch.yaml        # Terminal 4
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run easyfleet_fake_collaboration_deployment collaboration_mission_node
```

## What you should see

Only terminal output. The mission prints:

1. **Phase 0, discovery:** the capabilities of all three robots, with their descriptions.
2. **Phase 1:** `robot_1` and `robot_2` run navigation and perception, all four at once, for up to 10 s. Navigation finishes on its own after about 5 s, and perception is canceled at the cap.
3. **Phase 2:** `robot_3` runs navigation until it finishes, then manipulation.
4. `Mission complete`.

If a capability is missing, for example because you did not start `robot_3`, its step is reported as `not active, skipping` and the mission goes on.

## Key files

| File | Purpose |
|---|---|
| `launch/collaboration_launch.yaml` | Starts the Zenoh router, the three robots and the delayed mission. |
| `launch/robot_N_launch.yaml` | Starts one robot under `/robot_N`. Needs a router that is already running. |
| `config/robot_N/robot_params.yaml` | The capabilities this robot hosts. |
| `config/robot_N/*_params.yaml`, `*.json` | Mock parameters and the capability descriptions published on `/capabilities`. |
| `src/robot_node.cpp` | Process that hosts the capabilities. |
| `src/<package>/*_fake_capability.cpp` | Mock capability implementations. |
| `src/main_collaboration.cpp` | The two-phase mission (`collaboration_mission_node`). |

## Notes

- Each robot is a set of namespaced actions. Mission control looks robots up by the namespace that each capability announces, so adding a robot means adding a `config/robot_N/` folder and a `robot_N_launch.yaml`.
