#include <tf2_ros/static_transform_broadcaster.h>

#include <filesystem>

#include <CLI/CLI.hpp>
#include <boost/process.hpp>
#include <boost/process/args.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using tf2_msgs::msg::TFMessage;
namespace bp = boost::process;

TFMessage::UniquePtr read_static_tfs(const rclcpp::Node& node,
                                     const std::filesystem::path& bag_path) {
  const auto logger = node.get_logger();
  if (!std::filesystem::exists(bag_path)) {
    return nullptr;
  }

  rclcpp::get_logger("rosbag2_storage").set_level(rclcpp::Logger::Level::Warn);
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;

  const rclcpp::Serialization<TFMessage> serialization;
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  if (!reader) {
    return nullptr;
  }

  auto all_tfs = std::make_unique<TFMessage>();
  RCLCPP_DEBUG_STREAM(node.get_logger(), "reading static tfs from " << bag_path);

  reader->open(storage_options);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf_static"};
  reader->set_filter(filter);
  while (reader->has_next()) {
    const auto msg = reader->read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto tf_msg = std::make_shared<TFMessage>();
    serialization.deserialize_message(&serialized_msg, tf_msg.get());
    all_tfs->transforms.insert(all_tfs->transforms.end(),
                               tf_msg->transforms.begin(),
                               tf_msg->transforms.end());
  }

  RCLCPP_DEBUG_STREAM(node.get_logger(), "finished reading static tfs");
  if (all_tfs->transforms.empty()) {
    return nullptr;
  }

  return all_tfs;
}

int main(int argc, char** argv) {
  CLI::App app;
  argv = app.ensure_utf8(argv);
  app.allow_extras();

  // NOTE(nathan) for whatever reason, this doesn't get handled correctly when we use
  // the -- separator and multiple bags, so I give up
  std::string bag_path;
  app.add_option("bag_path", bag_path)->required()->check(CLI::ExistingPath);

  std::string message;
  app.add_option("-m,--message", message, "fake arg");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  std::vector<std::string> cmd_args{"bag", "play", bag_path};
  bool added_exclude = false;
  for (const auto& arg : app.remaining()) {
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

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tf_republisher");
  auto tf_msg = read_static_tfs(*node, bag_path);
  if (tf_msg) {
     auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
     broadcaster->sendTransform(tf_msg->transforms);
  }

  bp::child child(bp::search_path("ros2"), bp::args(cmd_args));
  rclcpp::spin(node);
  rclcpp::shutdown();

  child.wait();
  return child.exit_code();
}
