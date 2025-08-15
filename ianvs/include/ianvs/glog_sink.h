#pragma once
#include <glog/logging.h>

#include <rclcpp/logger.hpp>

#include "ianvs/node_handle.h"

namespace ianvs {

struct RosGlogSink {
  explicit RosGlogSink(const rclcpp::Logger& logger);
  explicit RosGlogSink(const NodeHandle& nh);
  ~RosGlogSink();

  struct Impl : google::LogSink {
    explicit Impl(const rclcpp::Logger& logger);
    void send(google::LogSeverity severity,
              const char* /*full_filename*/,
              const char* base_filename,
              int line,
              const struct ::tm* /*time*/,
              const char* message,
              size_t message_len) override;

    rclcpp::Logger logger_;
  };

  std::unique_ptr<Impl> impl_;
};

}  // namespace ianvs
