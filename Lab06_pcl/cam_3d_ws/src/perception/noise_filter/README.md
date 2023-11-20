# NOISE_FILTER
ROS2 node for PhoXi camera and RGB camera data collection.

## Requirements
* [PhoXi Camera ROS2](https://github.com/PPI-PUT/phoxi_camera_ros2)
* Custom RGB Camera
* PCL (tested with 1.12)
* OpenCV (tested with 4.5.4)

## Installation & building
```
sudo apt-get update && sudo apt-get install python3-vcstool
cd $HOME
mkdir -p phoxi_ws/src && cd phoxi_ws
git clone https://github.com/PPI-PUT/phoxi_camera_ros2 src/phoxi_camera_ros2
vcs import < default.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to noise_filter --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On
```

## Usage
1. Connect cameras.
2. Configure param file (noise_filter/param/defaults.param.yaml). Make sure the provided directory as `save_dir` already exists!
3. Run Phoxi and your RGB camera nodes. Phoxi node must be configured as services server (`trigger_mode: software`).
4. Validate RGB camera topic - see remappings options in `launch/noise_filter.launch.py`
5. Run noise_filter
```
source install/setup.bash
ros2 launch noise_filter noise_filter.launch.py
```
Service call:
```
ros2 service call /noise_filter/noise_filter/collect_data std_srvs/srv/Empty "{}"
```
See output files in your directory (which was definied in .yaml file).
