# ROS2 LiDAR Timestamp Bridge

This workspace contains a small ROS 2 bridge that forwards LiDAR message timestamps to shared memory for the camera synchronization workspace.

The `lidar_stamp_bridge_node` subscribes to `/lidar_points`, reads `msg->header.stamp` from each `sensor_msgs/msg/PointCloud2` message, and writes the timestamp plus receive metadata into `/dev/shm/lidar_stamp.bin`. The camera node in `ROS2-Camera-Sync-dominik` reads this mmap file and performs the actual camera-to-LiDAR timestamp matching.

This bridge does not match camera frames with LiDAR frames. It only prepares recent LiDAR timestamps for the camera workspace.

## Build

```bash
cd ROS2-LIDAR-CAMERA-BRIDGE
colcon build
source install/setup.bash
```

## Run Directly

```bash
ros2 run lidar_stamp_bridge lidar_stamp_bridge_node --ros-args \
  -p topic:=/lidar_points \
  -p mmap_path:=/dev/shm/lidar_stamp.bin
```

## Run With Launch

```bash
ros2 launch lidar_stamp_bridge lidar_stamp_bridge.launch.py
```

The launch file loads:

```text
src/lidar_stamp_bridge/config/lidar_stamp_bridge.yaml
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

## Verify

Check that LiDAR messages are arriving:

```bash
ros2 topic hz /lidar_points
```

Start the bridge and watch its log. Every received cloud should produce a line with `lidar_seq`, `ros_header_ns`, `host_ns`, and `frame_id`.
