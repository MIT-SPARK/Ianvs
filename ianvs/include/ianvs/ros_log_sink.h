#pragma once

#include <spdlog/sinks/sink.h>
#include <spdlog/spdlog.h>

#include <rclcpp/logger.hpp>

namespace ianvs {

//! Create a ROS sink to combine with other spdlog destinations
spdlog::sink_ptr makeRosSink(rclcpp::Logger logger);

//! Flush and replace a logger's sinks with ROS output while workers are stopped
void setRosSink(spdlog::logger& target, rclcpp::Logger logger);

}  // namespace ianvs
