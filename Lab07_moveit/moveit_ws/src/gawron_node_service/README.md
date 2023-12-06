# gawron_node_service
ROS2 node for service handling, plan and execute panda arm moving.

## Installation and Requirements

### Requirements

* Moveit (tested with 2.5.5-Alpha)
* tf2 (tested with 0.25.5)

## Installation

```bash
cd $HOME
cd moveit_ws/
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to gawron_node_service
source install/setup.bash
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

1. Configure param file (gawron_node_service/config/gawron_node_service.param.yaml). "planner_id" is for the planners types. Change if you want.

Rviz launch:
```bash
ros2 launch moveit_demo moveit_demo.launch.py
```

Node launch:
```bash
ros2 launch gawron_node_service gawron_node_service.launch.py planner_id_selection:=1
```
"planner_id_selection" is not reuired. If not passed default value is equal 1 (RRTkConfigDefault). List of planners:
- KPIECEkConfigDefault      #0
- RRTkConfigDefault         #1
- RRTConnectkConfigDefault  #2
- RRTstarkConfigDefault     #3
- TRRTkConfigDefault        #4
- PRMkConfigDefault         #5
- PRMstarkConfigDefault     #6

Service call:
```bash
ros2 service call /custom_service gawron_service/srv/CustomService "{position: {x: 0.3, y: 0.2, z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 1.57}}"
```
position - final position of the end effector
orientation - final orientation of the end effector

## API

### Input

| Name                   | Type                    | Description                               |
| ---------------------- | ----------------------- | ----------------------------------------- |
| `planner_id_selection` | int                     | Number representing the selected planner. |

### Services and Actions

| Name             | Type                               | Description                |
| ---------------- | ---------------------------------- | -------------------------- |
| `custom_service` | gawron_service::srv::CustomService | Service for Moveit handle. |

### Parameters

| Name         | Type | Description                              |
| ------------ | ---- | ---------------------------------------- |
| `planner_id` | str  | Planner type (e.g., "RRTkConfigDefault").|
