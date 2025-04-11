#include <tf2_ros/static_transform_broadcaster.h>

#include <filesystem>
#include <regex>

#include <CLI/CLI.hpp>
#include <boost/process.hpp>
#include <boost/process/args.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using Transform = geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;

namespace bp = boost::process;

struct FrameTransform {
  FrameTransform(const std::string& expr, const std::string& sub)
      : expr(expr), sub(sub) {}

  static FrameTransform prepend(const std::string& prefix) {
    return {".*", prefix + "$&"};
  }

  static FrameTransform exclude(const std::string& expr) { return {expr, ""}; }

  std::string apply(const std::string& frame_id) const {
    std::regex re(expr);
    return std::regex_replace(frame_id, re, sub);
  }

  const std::string expr;
  const std::string sub;
};

std::string apply_transforms(const std::vector<FrameTransform>& transforms,
                             const std::string& frame_id) {
  std::string result = frame_id;
  for (const auto& transform : transforms) {
    result = transform.apply(result);
    if (result.empty()) {
      return "";
    }
  }

  return result;
}

std::vector<Transform> read_static_tfs(
    const rclcpp::Node& node,
    const std::filesystem::path& bag_path,
    const std::vector<FrameTransform>& id_transforms) {
  const auto logger = node.get_logger();
  if (!std::filesystem::exists(bag_path)) {
    return {};
  }

  rclcpp::get_logger("rosbag2_storage").set_level(rclcpp::Logger::Level::Warn);
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;

  const rclcpp::Serialization<TFMessage> serialization;
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  if (!reader) {
    return {};
  }

  RCLCPP_DEBUG_STREAM(node.get_logger(), "reading static tfs from " << bag_path);
  std::map<std::string, std::map<std::string, Transform>> pose_map;
  reader->open(storage_options);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf_static"};
  reader->set_filter(filter);
  while (reader->has_next()) {
    const auto msg = reader->read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto tf_msg = std::make_shared<TFMessage>();
    serialization.deserialize_message(&serialized_msg, tf_msg.get());
    for (auto& tf : tf_msg->transforms) {
      const auto parent_id = apply_transforms(id_transforms, tf.header.frame_id);
      const auto child_id = apply_transforms(id_transforms, tf.child_frame_id);
      if (parent_id.empty() || child_id.empty()) {
        RCLCPP_WARN_STREAM(node.get_logger(),
                           "dropping filtered transform '" << tf.header.frame_id
                                                           << "_T_" << tf.child_frame_id
                                                           << "'");
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

      RCLCPP_WARN_STREAM(
          node.get_logger(),
          "dropping repeated tf: " << parent_id << "_T_" << child_id << "!");
    }
  }

  RCLCPP_DEBUG_STREAM(node.get_logger(), "finished reading static tfs");
  std::vector<Transform> poses;
  for (const auto& [parent, children] : pose_map) {
    for (const auto& [child, pose] : children) {
      poses.push_back(pose);
    }
  }

  return poses;
}

std::vector<std::string> get_bag_args(const std::string& bag_path,
                                      const std::vector<std::string>& remaining) {
  std::vector<std::string> cmd_args{"bag", "play", bag_path};
  bool added_exclude = false;
  for (const auto& arg : remaining) {
    if (arg == "--") {
      continue;
    }

    cmd_args.push_back(arg);
    if (arg == "--exclude-topics") {
      cmd_args.push_back("/tf_static");
      added_exclude = true;
    }
  }

  if (!added_exclude) {
    cmd_args.push_back("--exclude-topics");
    cmd_args.push_back("/tf_static");
  }

  return cmd_args;
}

int main(int argc, char** argv) {
  CLI::App app;
  argv = app.ensure_utf8(argv);
  app.allow_extras();

  // NOTE(nathan) for whatever reason, this doesn't get handled correctly when we use
  // the -- separator and multiple bags, so I give up
  std::string bag_path;
  app.add_option("bag_path", bag_path)->required()->check(CLI::ExistingPath);
  std::string prefix;
  app.add_option("-p,--prefix", prefix);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  std::vector<FrameTransform> frame_transforms;
  if (!prefix.empty()) {
    frame_transforms.push_back(FrameTransform::prepend(prefix));
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tf_republisher");
  auto transforms = read_static_tfs(*node, bag_path, frame_transforms);
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  if (!transforms.empty()) {
    broadcaster->sendTransform(transforms);
  }

  const auto cmd_args = get_bag_args(bag_path, app.remaining());
  bp::child child(bp::search_path("ros2"), bp::args(cmd_args));
  rclcpp::spin(node);
  rclcpp::shutdown();

  child.wait();
  return child.exit_code();
}
