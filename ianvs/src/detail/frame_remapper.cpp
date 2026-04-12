#include "ianvs/detail/frame_remapper.h"

#include <rclcpp/logging.hpp>

namespace ianvs {
namespace {

inline std::string frame_name(const std::string& dest, const std::string& src) {
  return "'" + dest + "_T_" + src + "'";
}

}  // namespace

using detail::StringTransform;

FrameRemapper::FrameRemapper(const Config& config, const rclcpp::Logger* logger) {
  if (!config.filter.empty()) {
    filter = std::regex(config.filter);
  }

  if (!config.keep.empty()) {
    keep = std::regex(config.keep);
  }

  if (!config.prefix.empty()) {
    transforms.push_back(
        StringTransform::from_arg(StringTransform::Type::Prefix, config.prefix, logger));
  }

  for (const auto& arg : config.substitutions) {
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Substitute, arg, logger));
  }
}

bool FrameRemapper::shouldKeep(const std::string& frame_id) const {
  if (!keep) {
    return true;
  }

  std::smatch match;
  return std::regex_match(frame_id, match, *keep);
}

bool FrameRemapper::shouldFilter(const std::string& frame_id) const {
  if (!filter) {
    return false;
  }

  std::smatch match;
  const bool should_filter = std::regex_match(frame_id, match, *filter);
  return should_filter;
}

std::string FrameRemapper::remapFrame(const std::string& frame_id) const {
  if (shouldFilter(frame_id) || !shouldKeep(frame_id)) {
    return "";
  }

  std::string result = frame_id;
  for (const auto& transform : transforms) {
    result = transform.apply(result);
    if (result.empty()) {
      return "";
    }
  }

  return result;
}

void FrameRemapper::updatePoseMap(const Msg& msg,
                                  PoseMap& pose_map,
                                  const rclcpp::Logger* logger) const {
  for (auto& tf : msg.transforms) {
    const auto parent_id = remapFrame(tf.header.frame_id);
    const auto child_id = remapFrame(tf.child_frame_id);
    if (parent_id.empty() || child_id.empty()) {
      if (logger) {
        const auto orig_name = frame_name(tf.header.frame_id, tf.child_frame_id);
        RCLCPP_INFO_STREAM(*logger, "Dropping filtered transform " << orig_name);
      }

      continue;
    }

    auto parent = pose_map.find(parent_id);
    if (parent == pose_map.end()) {
      parent = pose_map.emplace(parent_id, std::map<std::string, Transform>()).first;
    }

    auto& children = parent->second;
    auto child = children.find(child_id);
    if (child == children.end()) {
      auto iter = children.emplace(child_id, tf).first;
      iter->second.header.frame_id = parent_id;
      iter->second.child_frame_id = child_id;
      continue;
    }

    if (logger) {
      const auto remapped_name = frame_name(parent_id, child_id);
      RCLCPP_INFO_STREAM(*logger, "Dropping repeated tf " << remapped_name);
    }
  }
}

}  // namespace ianvs
