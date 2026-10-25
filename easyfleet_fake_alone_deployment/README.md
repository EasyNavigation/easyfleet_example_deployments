# easyfleet_fake_alone_deployment

The smallest EasyFleet example: a single robot, `robot_1`, with fake (mock) navigation, perception and manipulation capabilities, plus a mission node that discovers them and runs them. Nothing moves and there is no simulator. Progress is simulated on timers, so you can see the whole capability lifecycle in seconds: announcement on `/capabilities`, discovery, goals, feedback, results, preemption and cancel. Start here. [easyfleet_fake_collaboration_deployment](../easyfleet_fake_collaboration_deployment) adds more robots, and the `easyfleet_easynav_*` examples replace the fake navigation with real EasyNav in Gazebo.

See the [top-level README](../README.md) for the full list of examples.

## What runs where

| Process | Namespace | What it is |
|---|---|---|
| `robot_node` | `/robot_1` | One process that hosts the three capabilities as lifecycle nodes. It configures and activates them, and each one announces itself on `/capabilities`. |
| `alone_mission_node` | (none) | Mission control. Starts 3 s after the robot. |
| `rmw_zenohd` | (none) | Zenoh router, started by `alone_launch.yaml`. |

Capabilities of `robot_1`:

- `navigation` (`/robot_1/navigation`): simulates 5 s of travel, with `distance_remaining` going down from 5 m.
- `manipulation` (`/robot_1/manipulation`): simulates a 3 s trajectory.
- `perception` (`/robot_1/perception`): streams random detections of the class `gato` at 20 Hz until the goal is canceled, and rejects any other class.

## How to run

Build the workspace as described in the [top-level README](../README.md), then run:

```bash
source install/setup.bash
ros2 launch easyfleet_fake_alone_deployment alone_launch.yaml
```

This starts the Zenoh router, the robot and, after 3 s, the mission. `RMW_IMPLEMENTATION=rmw_zenoh_cpp` is set by the launch files.

To run the robot and the mission separately, start a router first:

```bash
# Terminal 1
ros2 run rmw_zenoh_cpp rmw_zenohd
# Terminal 2
ros2 launch easyfleet_fake_alone_deployment robot_1_launch.yaml
# Terminal 3
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run easyfleet_fake_alone_deployment alone_mission_node
```

## What you should see

Only terminal output. The mission prints:

1. **Phase 1, discovery:** a summary line and the full JSON description of each of `robot_1`'s three capabilities.
2. **Phase 2, sequential execution:** navigation, then manipulation, then perception. Each one runs until it finishes or for at most 10 s, and prints its feedback. Perception never finishes on its own, so it is canceled at 10 s.
3. **Phase 3, parallel execution:** the three capabilities run at the same time, with the same 10 s cap.
4. `Mission complete`.

## Key files

| File | Purpose |
|---|---|
| `launch/alone_launch.yaml` | Starts the Zenoh router, the robot and the delayed mission. |
| `launch/robot_1_launch.yaml` | Starts `robot_node` under `/robot_1`. Can be run on its own if a router is already running. |
| `config/robot_1/robot_params.yaml` | Which capabilities the robot hosts (`capabilities`) and the folder their JSON files are in (`config_subdir`). |
| `config/robot_1/*_params.yaml` | Mock parameters: preemption, durations, feedback periods, target class. |
| `config/robot_1/*.json` | Capability descriptions, published as they are on `/capabilities`. |
| `src/robot_node.cpp` | Creates, configures and activates the capabilities, then spins them. |
| `src/<package>/*_fake_capability.cpp` | Mock implementations of the `easyfleet_core` action-server bases. |
| `src/main_alone.cpp` | The mission (`alone_mission_node`), built on `easyfleet_mission_manager`. |

## Notes

- The mission only talks to the capabilities through their ROS actions. It does not link against their implementation.
- Mission control waits 3 s before discovery so the robot has time to announce itself. If you start the mission by hand before the robot is active, it finds nothing.
