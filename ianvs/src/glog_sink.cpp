#include "ianvs/glog_sink.h"

#include <rclcpp/logging.hpp>

namespace ianvs {

RosGlogSink::RosGlogSink(const rclcpp::Logger& logger)
    : impl_(std::make_unique<Impl>(logger)) {
  google::AddLogSink(impl_.get());
}

RosGlogSink::RosGlogSink(const NodeHandle& nh) : RosGlogSink(nh.logger()) {}

RosGlogSink::~RosGlogSink() { google::RemoveLogSink(impl_.get()); }

RosGlogSink::Impl::Impl(const rclcpp::Logger& logger) : logger_(logger) {}

void RosGlogSink::Impl::send(google::LogSeverity severity,
                             const char* /*full_filename*/,
                             const char* base_filename,
                             int line,
                             const struct ::tm* /*time*/,
                             const char* message,
                             size_t message_len) {
  std::stringstream ss;
  ss << "[" << base_filename << ":" << line << "] "
     << std::string(message, message_len);
  switch (severity) {
    case google::GLOG_WARNING:
      RCLCPP_WARN_STREAM(logger_, ss.str());
      break;
    case google::GLOG_ERROR:
      RCLCPP_ERROR_STREAM(logger_, ss.str());
      break;
    case google::GLOG_FATAL:
      RCLCPP_FATAL_STREAM(logger_, ss.str());
      break;
    case google::GLOG_INFO:
    default:
      RCLCPP_INFO_STREAM(logger_, ss.str());
      break;
  }
}

}  // namespace ianvs
