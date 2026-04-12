#pragma once
#include <optional>

#include <tf2_msgs/msg/tf_message.hpp>

#include "ianvs/detail/string_transforms.h"

namespace ianvs {

struct FrameRemapper {
  using Transform = geometry_msgs::msg::TransformStamped;
  using Msg = tf2_msgs::msg::TFMessage;
  using PoseMap = std::map<std::string, std::map<std::string, Transform>>;

  struct Config {
    std::string prefix;
    std::string filter;
    std::string keep;
    std::vector<std::string> substitutions;
  } const config;

  explicit FrameRemapper(const Config& config, const rclcpp::Logger* logger = nullptr);

  std::string remapFrame(const std::string& frame_id) const;

  void updatePoseMap(const Msg& msg,
                     PoseMap& pose_map,
                     const rclcpp::Logger* logger = nullptr) const;

  bool shouldKeep(const std::string& frame_id) const;

  bool shouldFilter(const std::string& frame_id) const;

  std::vector<detail::StringTransform> transforms;
  std::optional<std::regex> keep;
  std::optional<std::regex> filter;
};

}  // namespace ianvs
