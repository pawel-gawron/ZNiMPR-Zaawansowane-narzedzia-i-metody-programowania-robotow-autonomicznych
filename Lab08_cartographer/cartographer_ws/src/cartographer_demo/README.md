# cartographer_demo
<!-- Required -->
<!-- Package description -->
cartographer_demo launch files.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to cartographer_demo
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->
Run launch file:
* Mapping
```bash
ros2 launch cartographer_demo demo.launch.py mapping:=True
```

 * Localization
```bash
ros2 launch cartographer_demo demo.launch.py mapping:=False map_path:=/path/to/map.pbstream
```

## References / External links
<!-- Optional -->
