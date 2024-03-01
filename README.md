#  LiDAR Projection for ROS 2 Humble
This node publishes the projection of a point cloud from the LiDAR sensor to the camera frame. The projection is done using the TF2 or a calibration file from a camera extrinsic matrix.

# Getting Started
```bash
mkdir -p ros2_ws/src && cd ros2_ws/src
git clone git@github.com:knorrrr/lidar_projection_ros2.git
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

# Usage 
If you want to tune the external parameters (camera-LiDAR relationship) using TIER4/CalibrationTools' tunable_static_tf_broadcaster, you can follow these steps:
Install TIER4/CalibrationTools and ensure it is properly set up in your ROS 2 Humble environment.

```bash
source ../install/setup.bash
source <TIER4_calibrationtools>/install/setup.bash
ros2 launch lidar_projection_ros2 tunable_tf_lidar_projection.launch.xml 
```

If you want to load external parameters from a YAML file made by OpenCV, you can follow these steps:
```bash
source ../install/setup.bash
 ros2 launch lidar_projection_ros2 matrix_lidar_projection.launch.xml 
```

# IO
| Name                | Type     | Description                                          | Default                         |
| ------------------- | -------- | ---------------------------------------------------- |-------------------------------- |
| `lidar_topic`       | `String` | Topic for the LiDAR data input                       | `/vlp32c/velodyne_points`       |
| `input_image_topic` | `String` | Topic for the input image data to fuse an projected lidar image                       | `/davis/image_raw`              |
| `output_image_topic`| `String` | Topic for the output image data                      | `/davis/lidar_projection_image` |
| `camera_yaml_file`  | `String` | Path to the YAML file containing camera parameters   | `/path/to/camera.yaml`          |
| `use_tf`            | `bool`   | Flag indicating whether to use TF for transformation | `true`       |
| `parent_tf`         | `String` | Parent TF frame name                                 | `vlp32c`    |
| `child_tf`          | `String` | Child TF frame name                                  | `davis`     |
