# EasyFleet example deployments

[![rolling](https://github.com/EasyNavigation/easyfleet_example_deployments/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/EasyNavigation/easyfleet_example_deployments/actions/workflows/rolling.yaml)

Example deployments of [EasyFleet](https://github.com/EasyNavigation/EasyFleet), from mocked capabilities with no simulator to fleets running real [EasyNavigation](https://easynavigation.github.io/) navigation in Gazebo.

Web: [https://easynavigation.github.io/easyfleet_web/](https://easynavigation.github.io/easyfleet_web/)

## 📦 Packages

| Package | Description |
|-------------|-------------|
| [**easyfleet_fake_alone_deployment**](easyfleet_fake_alone_deployment) | One robot with fake navigation, perception and manipulation capabilities. No simulator needed. |
| [**easyfleet_fake_collaboration_deployment**](easyfleet_fake_collaboration_deployment) | Three robots with fake capabilities working together. No simulator needed. |
| [**easyfleet_easynav_deployment**](easyfleet_easynav_deployment) | One robot with the EasyNav-backed navigation capability (BehaviorTree.CPP). |
| [**easyfleet_easynav_collaboration_deployment**](easyfleet_easynav_collaboration_deployment) | Two robots running real EasyNav navigation, plus fake perception and manipulation. |
| [**easyfleet_easynav_collaboration_simple_api_deployment**](easyfleet_easynav_collaboration_simple_api_deployment) | The same two-robot scenario written with the simple `Deployment`/`SimpleController` API. |
| [**easyfleet_easynav_collaboration_simple_api_deployment_py**](easyfleet_easynav_collaboration_simple_api_deployment_py) | Python version of the simple API scenario. |
| [**easyfleet_easynav_collaboration_zenoh_deployment**](easyfleet_easynav_collaboration_zenoh_deployment) | The simple API scenario over `rmw_zenoh`, one Zenoh island per robot plus a central one. |
| [**easyfleet_easynav_collaboration_fastdds_deployment**](easyfleet_easynav_collaboration_fastdds_deployment) | The simple API scenario over Fast DDS, isolated per robot with DDS Router. |
| [**easyfleet_plansys2_warehouse_deployment**](easyfleet_plansys2_warehouse_deployment) | Four Kobukis in a large warehouse, driven by PlanSys2: the behavior tree of each plan action calls the robots' capabilities. |
| [**easyfleet_easynav_farm_deployment**](easyfleet_easynav_farm_deployment) | One Summit robot navigating outdoors on a NavMap, localized with GPS. |

## Build

```bash
mkdir -p ~/easyfleet_ws/src && cd ~/easyfleet_ws/src
git clone https://github.com/EasyNavigation/easyfleet_example_deployments.git -b rolling
vcs import < easyfleet_example_deployments/easyfleet_example_deployments.repos
cd ~/easyfleet_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## License

Apache-2.0 (see `LICENSE`).
