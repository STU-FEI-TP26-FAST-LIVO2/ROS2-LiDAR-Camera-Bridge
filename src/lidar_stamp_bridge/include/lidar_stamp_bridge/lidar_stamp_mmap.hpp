#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <stdexcept>
#include <vector>
#include <optional>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace lidar_stamp_bridge
{

constexpr uint32_t kLidarRingMagic = 0x4c53594e; // 'LSYN'
constexpr uint32_t kLidarRingVersion = 1;
constexpr uint32_t kLidarFlagValid = 0x1;

#pragma pack(push, 1)
struct LidarRingHeader
{
  uint32_t magic;
  uint32_t version;
  uint32_t capacity;
  uint32_t record_size;
  uint64_t write_index;
  uint64_t reserved[8];
};

struct LidarRingRecord
{
  uint32_t flags;
  uint32_t reserved0;
  uint64_t lidar_ros_header_ns;
  uint64_t lidar_host_receive_ns;
  uint64_t lidar_seq;
  char frame_id[64];
};
#pragma pack(pop)

class LidarStampWriter
{
public:
  LidarStampWriter(const std::string & path, uint32_t capacity)
  : path_(path), capacity_(capacity)
  {
    if (capacity_ == 0) {
      throw std::runtime_error("lidar mmap capacity must be > 0");
    }

    const size_t total_size = sizeof(LidarRingHeader) + capacity_ * sizeof(LidarRingRecord);

    fd_ = ::open(path_.c_str(), O_RDWR | O_CREAT, 0666);
    if (fd_ < 0) {
      throw std::runtime_error("open failed: " + path_);
    }

    if (ftruncate(fd_, static_cast<off_t>(total_size)) != 0) {
      ::close(fd_);
      throw std::runtime_error("ftruncate failed: " + path_);
    }

    mapping_ = ::mmap(nullptr, total_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
      ::close(fd_);
      throw std::runtime_error("mmap failed: " + path_);
    }

    total_size_ = total_size;
    header_ = static_cast<LidarRingHeader *>(mapping_);
    records_ = reinterpret_cast<LidarRingRecord *>(
      static_cast<uint8_t *>(mapping_) + sizeof(LidarRingHeader));

    if (header_->magic != kLidarRingMagic ||
        header_->version != kLidarRingVersion ||
        header_->capacity != capacity_ ||
        header_->record_size != sizeof(LidarRingRecord))
    {
      std::memset(mapping_, 0, total_size_);
      header_->magic = kLidarRingMagic;
      header_->version = kLidarRingVersion;
      header_->capacity = capacity_;
      header_->record_size = sizeof(LidarRingRecord);
      header_->write_index = 0;
    }
  }

  ~LidarStampWriter()
  {
    if (mapping_ && mapping_ != MAP_FAILED) {
      ::msync(mapping_, total_size_, MS_SYNC);
      ::munmap(mapping_, total_size_);
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void write(uint64_t ros_ns, uint64_t host_ns, uint64_t seq, const std::string & frame_id)
  {
    const uint64_t index = header_->write_index;
    auto & rec = records_[index % capacity_];

    std::memset(&rec, 0, sizeof(rec));
    rec.flags = kLidarFlagValid;
    rec.lidar_ros_header_ns = ros_ns;
    rec.lidar_host_receive_ns = host_ns;
    rec.lidar_seq = seq;
    std::strncpy(rec.frame_id, frame_id.c_str(), sizeof(rec.frame_id) - 1);

    header_->write_index = index + 1;
  }

private:
  std::string path_;
  uint32_t capacity_{0};
  int fd_{-1};
  void * mapping_{nullptr};
  size_t total_size_{0};
  LidarRingHeader * header_{nullptr};
  LidarRingRecord * records_{nullptr};
};

}  // namespace lidar_stamp_bridge