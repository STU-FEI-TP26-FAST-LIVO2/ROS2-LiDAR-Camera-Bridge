# ROS 2 LiDAR Timestamp Bridge

This workspace contains a small ROS 2 package that forwards LiDAR message timestamps to shared memory for the camera synchronization workspace.

The `lidar_stamp_bridge_node` subscribes to `/lidar_points`, reads `msg->header.stamp` from each `sensor_msgs/msg/PointCloud2` message, and writes the timestamp plus receive metadata into `/dev/shm/lidar_stamp.bin`. The camera node in `ROS2-Camera-Sync-dominik` reads this mmap file and performs the actual camera-to-LiDAR timestamp matching.

This bridge does not match camera frames with LiDAR frames. It only prepares recent LiDAR timestamps for the camera workspace.

## Package

```text
src/lidar_stamp_bridge
```

Installed files:

- executable: `lidar_stamp_bridge_node`
- launch file: `lidar_stamp_bridge.launch.py`
- config files: `lidar_stamp_bridge.yaml`, `cyclonedds.xml`

## Requirements

- ROS 2 with `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- Cyclone DDS runtime when using the provided launch file:

```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```

## Build

```bash
cd ROS2-LIDAR-CAMERA-BRIDGE
colcon build
source install/setup.bash
```

If large LiDAR clouds are dropped or Cyclone DDS reports a small receive buffer, increase the kernel receive buffer before running:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
```

## Run Directly

Direct `ros2 run` does not load the launch-file DDS environment. If the LiDAR publisher also uses Cyclone DDS, source the helper script first:

```bash
source src/lidar_stamp_bridge/scripts/source_cyclone_env.sh
```

Then start the node:

```bash
ros2 run lidar_stamp_bridge lidar_stamp_bridge_node --ros-args \
  -p topic:=/lidar_points \
  -p mmap_path:=/dev/shm/lidar_stamp.bin
```

## Run With Launch

```bash
ros2 launch lidar_stamp_bridge lidar_stamp_bridge.launch.py
```

The launch file loads the parameters from:

```text
src/lidar_stamp_bridge/config/lidar_stamp_bridge.yaml
```

It also sets:

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file://<installed-package-share>/config/cyclonedds.xml
```

## Parameters

```yaml
lidar_stamp_bridge_node:
  ros__parameters:
    topic: "/lidar_points"
    mmap_path: "/dev/shm/lidar_stamp.bin"
    mmap_slots: 512
```

- `topic`: PointCloud2 topic to subscribe to.
- `mmap_path`: shared memory file written by this bridge and read by the camera node.
- `mmap_slots`: number of timestamp records kept in the shared memory ring buffer.

The subscription uses reliable QoS with a queue depth of 25.

## Shared Memory Data

The node creates or resizes the mmap file at `mmap_path`. The file contains:

- header with magic `LSYN`, version `1`, capacity, record size, and `write_index`
- ring-buffer records with:
  - validity flags
  - LiDAR ROS header timestamp in nanoseconds
  - host receive timestamp in nanoseconds
  - local LiDAR sequence number
  - `frame_id`

If the existing mmap header does not match the expected format or configured capacity, the file is cleared and reinitialized.

## Verify

Check that LiDAR messages are arriving:

```bash
ros2 topic hz /lidar_points
```

Start the bridge and watch its log. Every received cloud should produce a line with `lidar_seq`, `ros_header_ns`, `host_ns`, and `frame_id`.

You can also check that the mmap file exists:

```bash
ls -lh /dev/shm/lidar_stamp.bin
```
