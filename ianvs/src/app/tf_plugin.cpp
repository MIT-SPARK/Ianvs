#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <map>

#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ianvs/app/rosbag_play_plugins.h"
#include "ianvs/detail/string_transforms.h"

namespace ianvs {
namespace {

inline std::string frame_name(const std::string& dest, const std::string& src) {
  return "'" + dest + "_T_" + src + "'";
}

}  // namespace

using Transform = geometry_msgs::msg::TransformStamped;
using detail::StringTransform;
using tf2_msgs::msg::TFMessage;

using PoseMap = std::map<std::string, std::map<std::string, Transform>>;
using ArgVec = std::vector<std::string>;

struct FrameRemapper {
  FrameRemapper(const std::vector<StringTransform>& transforms) : transforms(transforms) {}
  std::string remapFrame(const std::string& frame_id) const;
  void updatePoseMap(const TFMessage& msg,
                     PoseMap& pose_map,
                     const rclcpp::Logger* logger = nullptr);

  const std::vector<StringTransform> transforms;
};

class TFPlugin : public RosbagPlayPlugin {
 public:
  struct Config {
    std::string prefix;
    std::string filter;
    std::vector<std::string> substitutions;
    bool filter_dynamic = false;

    std::vector<StringTransform> transforms(const rclcpp::Logger* logger = nullptr) const;
  } config;

  TFPlugin();

  void init(std::shared_ptr<rclcpp::Node> node) override;
  void add_options(CLI::App& app) override;
  void modify_playback(rosbag2_transport::PlayOptions& options) override;
  void on_start(rosbag2_cpp::Reader& reader, const rclcpp::Logger* logger = nullptr) override;
  void on_stop() override {}

  const rclcpp::Serialization<TFMessage> serialization;

 private:
  void callback(TFMessage::UniquePtr msg);
  void publishStaticTFs(const PoseMap& pose_map);

  std::string tf_topic_;
  std::unique_ptr<FrameRemapper> remapper_;
  rclcpp::Subscription<TFMessage>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
};

std::string FrameRemapper::remapFrame(const std::string& frame_id) const {
  std::string result = frame_id;
  for (const auto& transform : transforms) {
    result = transform.apply(result);
    if (result.empty()) {
      return "";
    }
  }

  return result;
}

void FrameRemapper::updatePoseMap(const TFMessage& msg,
                                  PoseMap& pose_map,
                                  const rclcpp::Logger* logger) {
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

std::vector<StringTransform> TFPlugin::Config::transforms(const rclcpp::Logger* logger) const {
  std::vector<StringTransform> transforms;
  if (!filter.empty()) {
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Filter, filter, logger));
  }

  if (!prefix.empty()) {
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Prefix, prefix, logger));
  }

  for (const auto& arg : substitutions) {
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Substitute, arg, logger));
  }

  return transforms;
}

TFPlugin::TFPlugin() {}

void TFPlugin::init(std::shared_ptr<rclcpp::Node> node) {
  tf_topic_ = node->get_node_topics_interface()->resolve_topic_name("~/_tf");
  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  sub_ = node->create_subscription<TFMessage>(
      tf_topic_,
      tf2_ros::DynamicListenerQoS(),
      std::bind(&TFPlugin::callback, this, std::placeholders::_1));
}

void TFPlugin::add_options(CLI::App& app) {
  app.add_option("-p,--prefix", config.prefix)
      ->take_last()
      ->description("prefix to apply to ALL frames");
  app.add_option("-f,--filter", config.filter)
      ->join('|')
      ->description("optional regex filter to drop frames (applied before prefix)");
  app.add_option("-s,--substitution", config.substitutions)
      ->description("apply substitution (match and substituion are separated by :)");
  app.add_flag("--filter-dynamic", config.filter_dynamic, "enable filtering /tf");
}

void TFPlugin::modify_playback(rosbag2_transport::PlayOptions& options) {
  // TODO(nathan) also double-check exclusion regex
  std::set<std::string> excluded(options.exclude_topics_to_filter.begin(),
                                 options.exclude_topics_to_filter.end());
  if (excluded.count("/tf_static")) {
    broadcaster_.reset();
  }

  options.exclude_topics_to_filter.push_back("/tf_static");
  if (config.filter_dynamic) {
    options.topic_remapping_options.push_back("--remap");
    options.topic_remapping_options.push_back("/tf:=" + tf_topic_);
  }
}

void TFPlugin::on_start(rosbag2_cpp::Reader& reader, const rclcpp::Logger* logger) {
  remapper_ = std::make_unique<FrameRemapper>(config.transforms(logger));
  if (!config.filter_dynamic) {
    dynamic_broadcaster_.reset();
  }

  if (!broadcaster_) {
    return;  // don't do work if we don't publish static transforms
  }

  PoseMap pose_map;
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf_static"};
  reader.set_filter(filter);
  while (reader.has_next()) {
    const auto msg = reader.read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto tf_msg = std::make_shared<TFMessage>();
    serialization.deserialize_message(&serialized_msg, tf_msg.get());
    remapper_->updatePoseMap(*tf_msg, pose_map, logger);
  }

  publishStaticTFs(pose_map);
}

void TFPlugin::callback(TFMessage::UniquePtr msg) {
  if (!remapper_ || !dynamic_broadcaster_) {
    return;
  }

  PoseMap pose_map;
  remapper_->updatePoseMap(*msg, pose_map);
  if (pose_map.empty()) {
    return;
  }

  std::vector<Transform> poses;
  for (const auto& [parent, children] : pose_map) {
    for (const auto& [child, pose] : children) {
      poses.push_back(pose);
    }
  }

  dynamic_broadcaster_->sendTransform(poses);
}

void TFPlugin::publishStaticTFs(const PoseMap& pose_map) {
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

PLUGINLIB_EXPORT_CLASS(ianvs::TFPlugin, ianvs::RosbagPlayPlugin)
