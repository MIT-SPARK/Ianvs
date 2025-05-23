#include <tf2_ros/static_transform_broadcaster.h>

#include <map>

#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ianvs/app/rosbag_play_plugins.h"
#include "ianvs/detail/string_transforms.h"

namespace ianvs {

using Transform = geometry_msgs::msg::TransformStamped;
using detail::StringTransform;
using tf2_msgs::msg::TFMessage;

namespace {

struct FrameTransforms {
  struct Config {
    std::string prefix;
    std::string filter;
    std::vector<std::string> substitutions;
  };

  explicit FrameTransforms(const std::vector<StringTransform>& transforms)
      : transforms(transforms) {}

  static FrameTransforms fromConfig(const Config& config,
                                    const rclcpp::Logger* logger) {
    std::vector<StringTransform> transforms;
    if (!config.filter.empty()) {
      transforms.push_back(StringTransform::from_arg(
          StringTransform::Type::Filter, config.filter, logger));
    }

    if (!config.prefix.empty()) {
      transforms.push_back(StringTransform::from_arg(
          StringTransform::Type::Prefix, config.prefix, logger));
    }

    for (const auto& arg : config.substitutions) {
      transforms.push_back(
          StringTransform::from_arg(StringTransform::Type::Substitute, arg, logger));
    }

    return FrameTransforms(transforms);
  }

  std::string apply(const std::string& frame_id) const {
    std::string result = frame_id;
    for (const auto& transform : transforms) {
      result = transform.apply(result);
      if (result.empty()) {
        return "";
      }
    }

    return result;
  }

  std::vector<StringTransform> transforms;
};

inline std::string frame_name(const std::string& dest, const std::string& src) {
  return "'" + dest + "_T_" + src + "'";
}

}  // namespace

class StaticTfPlugin : public RosbagPlayPlugin {
 public:
  StaticTfPlugin();

  void init(std::shared_ptr<rclcpp::Node> node) override;
  void add_options(CLI::App& app) override;
  void on_start(rosbag2_cpp::Reader& reader,
                const rclcpp::Logger* logger = nullptr) override;
  void on_stop() override {}

 private:
  FrameTransforms::Config config;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

StaticTfPlugin::StaticTfPlugin() {}

void StaticTfPlugin::init(std::shared_ptr<rclcpp::Node> node) {
  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
}

void StaticTfPlugin::add_options(CLI::App& app) {
  app.add_option("-p,--prefix", config.prefix)
      ->take_last()
      ->description("prefix to apply to ALL frames");
  app.add_option("-f,--filter", config.filter)
      ->join('|')
      ->description("optional regex filter to drop frames (applied before prefix)");
  app.add_option("-s,--substitution", config.substitutions)
      ->description("apply substitution (match and substituion are separated by :)");
}

void StaticTfPlugin::on_start(rosbag2_cpp::Reader& reader,
                              const rclcpp::Logger* logger) {
  const rclcpp::Serialization<TFMessage> serialization;
  const auto frame_transforms = FrameTransforms::fromConfig(config, logger);

  std::map<std::string, std::map<std::string, Transform>> pose_map;

  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf_static"};
  reader.set_filter(filter);
  while (reader.has_next()) {
    const auto msg = reader.read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto tf_msg = std::make_shared<TFMessage>();
    serialization.deserialize_message(&serialized_msg, tf_msg.get());
    for (auto& tf : tf_msg->transforms) {
      const auto parent_id = frame_transforms.apply(tf.header.frame_id);
      const auto child_id = frame_transforms.apply(tf.child_frame_id);
      if (parent_id.empty() || child_id.empty()) {
        if (logger) {
          const auto orig_name = frame_name(tf.header.frame_id, tf.child_frame_id);
          RCLCPP_INFO_STREAM(*logger, "Dropping filtered transform " << orig_name);
        }

        continue;
      }

      tf.header.frame_id = parent_id;
      tf.child_frame_id = child_id;

      auto parent = pose_map.find(parent_id);
      if (parent == pose_map.end()) {
        parent = pose_map.emplace(parent_id, std::map<std::string, Transform>()).first;
      }

      auto& children = parent->second;
      auto child = children.find(child_id);
      if (child == children.end()) {
        children.emplace(child_id, tf);
        continue;
      }

      if (logger) {
        const auto remapped_name = frame_name(parent_id, child_id);
        RCLCPP_INFO_STREAM(*logger, "Dropping repeated tf " << remapped_name);
      }
    }
  }

  std::vector<Transform> poses;
  for (const auto& [parent, children] : pose_map) {
    for (const auto& [child, pose] : children) {
      poses.push_back(pose);
    }
  }

  if (poses.empty() || !broadcaster_) {
    return;
  }

  broadcaster_->sendTransform(poses);
}

}  // namespace ianvs

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ianvs::StaticTfPlugin, ianvs::RosbagPlayPlugin)
