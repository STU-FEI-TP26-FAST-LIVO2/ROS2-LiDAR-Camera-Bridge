#include "lidar_stamp_bridge/lidar_stamp_bridge_node.hpp"

#include <functional>
#include <memory>

namespace lidar_stamp_bridge
{

LidarStampBridgeNode::LidarStampBridgeNode()
: Node("lidar_stamp_bridge_node")
{
  declare_parameter<std::string>("topic", "/lidar_points");
  declare_parameter<std::string>("mmap_path", "/dev/shm/lidar_stamp.bin");
  declare_parameter<int>("mmap_slots", 512);

  topic_ = get_parameter("topic").as_string();
  mmap_path_ = get_parameter("mmap_path").as_string();
  mmap_slots_ = get_parameter("mmap_slots").as_int();

  writer_ = std::make_unique<LidarStampWriter>(
    mmap_path_, static_cast<uint32_t>(mmap_slots_));

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_,
    build_qos(),
    std::bind(&LidarStampBridgeNode::point_cloud_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "LiDAR stamp bridge started. topic=%s mmap_path=%s mmap_slots=%d",
    topic_.c_str(), mmap_path_.c_str(), mmap_slots_);
}

rclcpp::QoS LidarStampBridgeNode::build_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(25)).reliable();
}

void LidarStampBridgeNode::point_cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  ++seq_;

  const uint64_t ros_ns =
    static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
    static_cast<uint64_t>(msg->header.stamp.nanosec);
  const uint64_t host_ns = static_cast<uint64_t>(this->now().nanoseconds());

  const uint64_t delta_host_ns = (prev_host_ns_ > 0) ? (host_ns - prev_host_ns_) : 0;
  const uint64_t delta_ros_ns = (prev_ros_ns_ > 0) ? (ros_ns - prev_ros_ns_) : 0;
  prev_host_ns_ = host_ns;
  prev_ros_ns_ = ros_ns;

  writer_->write(ros_ns, host_ns, seq_, msg->header.frame_id);

  RCLCPP_INFO(
    get_logger(),
    "lidar_seq=%lu ros_header_ns=%lu host_ns=%lu delta_host_ms=%.1f "
    "delta_ros_ms=%.1f frame_id=%s",
    static_cast<unsigned long>(seq_),
    static_cast<unsigned long>(ros_ns),
    static_cast<unsigned long>(host_ns),
    delta_host_ns / 1e6,
    delta_ros_ns / 1e6,
    msg->header.frame_id.c_str());
}

}  // namespace lidar_stamp_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_stamp_bridge::LidarStampBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
