#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <map>

#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ianvs/app/rosbag_play_plugins.h"
#include "ianvs/detail/bag_topic_checker.h"
#include "ianvs/detail/string_transforms.h"

namespace ianvs {

using detail::StringTransform;

namespace {

inline std::string frame_name(const std::string& dest, const std::string& src) {
  return "'" + dest + "_T_" + src + "'";
}

}  // namespace

using Transform = geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;

using PoseMap = std::map<std::string, std::map<std::string, Transform>>;
using ArgVec = std::vector<std::string>;

struct FrameRemapper;

class TFPlugin : public RosbagPlayPlugin {
 public:
  struct Config {
    std::string prefix;
    std::string filter;
    std::string keep;
    std::vector<std::string> substitutions;
    bool filter_dynamic = false;
  } config;

  TFPlugin();

  void init(std::shared_ptr<rclcpp::Node> node) override;
  void add_options(CLI::App& app) override;
  void on_start(rosbag2_cpp::Reader& reader,
                rosbag2_transport::PlayOptions& options,
                const rclcpp::Logger* logger = nullptr) override;
  void on_stop() override {}

  const rclcpp::Serialization<TFMessage> serialization;

 private:
  void callback(TFMessage::UniquePtr msg);
  void publishStaticTFs(const PoseMap& pose_map);

  std::string tf_topic_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FrameRemapper> remapper_;
  rclcpp::Subscription<TFMessage>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
};

struct FrameRemapper {
  explicit FrameRemapper(const TFPlugin::Config& config, const rclcpp::Logger* logger = nullptr);

  std::string remapFrame(const std::string& frame_id) const;
  void updatePoseMap(const TFMessage& msg,
                     PoseMap& pose_map,
                     const rclcpp::Logger* logger = nullptr);

  bool shouldKeep(const std::string& frame_id) const;
  bool shouldFilter(const std::string& frame_id) const;

  std::vector<StringTransform> transforms;
  std::optional<std::regex> keep;
  std::optional<std::regex> filter;
};

FrameRemapper::FrameRemapper(const TFPlugin::Config& config, const rclcpp::Logger* logger) {
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

TFPlugin::TFPlugin() {}

void TFPlugin::init(std::shared_ptr<rclcpp::Node> node) {
  node_ = node;
  tf_topic_ = node_->get_node_topics_interface()->resolve_topic_name("~/_tf");
}

void TFPlugin::add_options(CLI::App& app) {
  app.add_option("-p,--prefix-frames", config.prefix)
      ->take_last()
      ->description("prefix to apply to ALL frames");
  app.add_option("-f,--filter-frames", config.filter)
      ->join('|')
      ->description("optional regex filter to drop frames (applied before prefix)");
  app.add_option("-k,--keep-frames", config.keep)
      ->join('|')
      ->description("optional regex filter to keep frames (applied before prefix)");
  app.add_option("-s,--frame-substitution", config.substitutions)
      ->description("apply substitution to frames (match and substituion are separated by :)");
  app.add_flag(
      "--filter-tf", config.filter_dynamic, "enable filtering /tf in addition to /tf_static");
}

void TFPlugin::on_start(rosbag2_cpp::Reader& reader,
                        rosbag2_transport::PlayOptions& options,
                        const rclcpp::Logger* logger) {
  remapper_ = std::make_unique<FrameRemapper>(config, logger);

  BagTopicChecker checker(options);
  if (config.filter_dynamic && checker.passes("/tf")) {
    options.topic_remapping_options.push_back("--remap");
    options.topic_remapping_options.push_back("/tf:=" + tf_topic_);
    dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    sub_ = node_->create_subscription<TFMessage>(
        tf_topic_,
        tf2_ros::DynamicListenerQoS(),
        std::bind(&TFPlugin::callback, this, std::placeholders::_1));
  }

  std::set<std::string> excluded(options.exclude_topics_to_filter.begin(),
                                 options.exclude_topics_to_filter.end());
  if (!checker.passes("/tf_static")) {
    return;
  }

  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
  options.exclude_topics_to_filter.push_back("/tf_static");

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
