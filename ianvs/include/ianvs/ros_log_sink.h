#pragma once

#include <spdlog/spdlog.h>

#include <rclcpp/logger.hpp>

namespace ianvs {

//! Create a sink that forwards to the ROS logger's spdlog child at DEBUG level
spdlog::sink_ptr makeRosSink(rclcpp::Logger logger);

//! Flush and replace existing sinks with ROS forwarding; configure between runs
void setRosSink(spdlog::logger& target, rclcpp::Logger logger);

}  // namespace ianvs
