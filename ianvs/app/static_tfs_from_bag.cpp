#include <filesystem>

#include <CLI/CLI.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ianvs/app/rosbag_play_plugins.h"
#include "ianvs/detail/frame_remapper.h"

using namespace std::chrono_literals;
using PluginVec = std::vector<std::shared_ptr<ianvs::RosbagPlayPlugin>>;

struct AppArgs {
  void add_to_app(CLI::App& app);

  std::filesystem::path bag;
  ianvs::FrameRemapper::Config remapper;
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("bag_path", bag)->required()->check(CLI::ExistingPath)->description("Bag to open");
}

int main(int argc, char** argv) {
  CLI::App app("Utility to save static TFS from a file");
  argv = app.ensure_utf8(argv);
  app.allow_extras();
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_to_app(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = args.bag;
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_opts);
  if (!reader) {
    return 1;
  }

  ianvs::FrameRemapper::PoseMap pose_map;
  const ianvs::FrameRemapper remapper(args.remapper);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf_static"};
  const rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;

  reader->open(storage_opts);
  reader->set_filter(filter);
  while (reader->has_next()) {
    const auto msg = reader->read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
    serialization.deserialize_message(&serialized_msg, tf_msg.get());
    remapper.updatePoseMap(*tf_msg, pose_map);
  }

  return 0;
}
