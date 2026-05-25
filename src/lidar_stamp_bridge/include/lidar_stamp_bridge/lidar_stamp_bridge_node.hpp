#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <string>

#include "lidar_stamp_bridge/lidar_stamp_mmap.hpp"

namespace lidar_stamp_bridge
{

class LidarStampBridgeNode : public rclcpp::Node
{
public:
  LidarStampBridgeNode();

private:
  static rclcpp::QoS build_qos();

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::string topic_{"/lidar_points"};
  std::string mmap_path_{"/dev/shm/lidar_stamp.bin"};
  int mmap_slots_{512};

  uint64_t seq_{0};
  uint64_t prev_host_ns_{0};
  uint64_t prev_ros_ns_{0};

  std::unique_ptr<LidarStampWriter> writer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace lidar_stamp_bridge
