#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <fstream>
#include <memory>
#include <string>
#include <functional>
#include <filesystem>
#include <cstdlib>

#include "lidar_stamp_bridge/lidar_stamp_mmap.hpp"

namespace fs = std::filesystem;

class LidarStampBridgeNode : public rclcpp::Node
{
public:
  LidarStampBridgeNode()
  : Node("lidar_stamp_bridge_node")
  {
    declare_parameter<std::string>("topic", "/lidar_points");
    declare_parameter<std::string>("mmap_path", "/dev/shm/lidar_stamp.bin");
    declare_parameter<std::string>("csv_path", "/home/jetson/ROS2-Camera-Sync-dominik/Matching/lidar_stamp_log.csv");
    declare_parameter<int>("mmap_slots", 128);

    // QoS parameters
    declare_parameter<bool>("sensor_data_qos", false);
    declare_parameter<int>("queue_depth", 25);
    declare_parameter<std::string>("reliability", "reliable");

    topic_      = get_parameter("topic").as_string();
    mmap_path_  = get_parameter("mmap_path").as_string();
    csv_path_   = get_parameter("csv_path").as_string();
    mmap_slots_ = get_parameter("mmap_slots").as_int();

    const bool        sensor_data_qos = get_parameter("sensor_data_qos").as_bool();
    const int         queue_depth     = get_parameter("queue_depth").as_int();
    const std::string reliability     = get_parameter("reliability").as_string();

    writer_ = std::make_unique<lidar_stamp_bridge::LidarStampWriter>(
      mmap_path_, static_cast<uint32_t>(mmap_slots_));

    open_csv_file();

    rclcpp::QoS qos = build_qos(sensor_data_qos, queue_depth, reliability);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_,
      qos,
      std::bind(&LidarStampBridgeNode::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "LidarStampBridge spusteny. topic=%s mmap_path=%s csv_path=%s mmap_slots=%d",
      topic_.c_str(), mmap_path_.c_str(), csv_path_.c_str(), mmap_slots_);

    RCLCPP_INFO(get_logger(),
      "QoS: sensor_data_qos=%s  queue_depth=%d  reliability=%s",
      sensor_data_qos ? "true" : "false",
      sensor_data_qos ? 5 : queue_depth,          // SensorDataQoS fixes depth=5
      sensor_data_qos ? "best_effort" : reliability.c_str());
  }

private:
  static rclcpp::QoS build_qos(bool sensor_data_qos, int queue_depth,
                                const std::string & reliability)
  {
    if (sensor_data_qos) {
      return rclcpp::SensorDataQoS();
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth));
    if (reliability == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    return qos;
  }

  static std::string expand_home(const std::string & path)
  {
    if (!path.empty() && path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        return std::string(home) + path.substr(1);
      }
    }
    return path;
  }

  void open_csv_file()
  {
    if (csv_path_.empty()) {
      RCLCPP_WARN(get_logger(), "CSV path is empty, CSV logging disabled");
      return;
    }

    std::string expanded_path = expand_home(csv_path_);

    try {
      fs::path csv_file_path(expanded_path);
      fs::path parent_dir = csv_file_path.parent_path();

      if (!parent_dir.empty() && !fs::exists(parent_dir)) {
        fs::create_directories(parent_dir);
        RCLCPP_INFO(get_logger(), "Created CSV directory: %s", parent_dir.c_str());
      }

      csv_.open(expanded_path, std::ios::out | std::ios::trunc);
      if (!csv_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", expanded_path.c_str());
      } else {
        csv_ << "seq,ros_header_ns,host_ns,delta_host_ns,delta_ros_ns,frame_id\n";
        csv_.flush();
        RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", expanded_path.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error setting up CSV logging: %s", e.what());
    }
  }

  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++seq_;

    const uint64_t ros_ns =
      static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(msg->header.stamp.nanosec);

    const uint64_t host_ns = static_cast<uint64_t>(this->now().nanoseconds());

    const uint64_t delta_host_ns = (prev_host_ns_ > 0) ? (host_ns - prev_host_ns_) : 0;
    const uint64_t delta_ros_ns  = (prev_ros_ns_  > 0) ? (ros_ns  - prev_ros_ns_)  : 0;
    prev_host_ns_ = host_ns;
    prev_ros_ns_  = ros_ns;

    writer_->write(ros_ns, host_ns, seq_, msg->header.frame_id);

    if (csv_.is_open()) {
      csv_ << seq_ << "," << ros_ns << "," << host_ns << ","
           << delta_host_ns << "," << delta_ros_ns << ","
           << msg->header.frame_id << "\n";
      csv_.flush();
    }

    RCLCPP_INFO(get_logger(),
      "lidar_seq=%lu ros_header_ns=%lu host_ns=%lu delta_host_ms=%.1f delta_ros_ms=%.1f frame_id=%s",
      (unsigned long)seq_,
      (unsigned long)ros_ns,
      (unsigned long)host_ns,
      delta_host_ns / 1e6,
      delta_ros_ns  / 1e6,
      msg->header.frame_id.c_str());
  }

  std::string topic_;
  std::string mmap_path_;
  std::string csv_path_;
  int mmap_slots_{128};
  uint64_t seq_{0};
  uint64_t prev_host_ns_{0};
  uint64_t prev_ros_ns_{0};

  std::unique_ptr<lidar_stamp_bridge::LidarStampWriter> writer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::ofstream csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarStampBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
