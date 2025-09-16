
#include <map>

#include <rosbag2_transport/reader_writer_factory.hpp>

#include "ianvs/app/rosbag_play_plugins.h"
#include "ianvs/detail/string_transforms.h"

namespace ianvs {

using detail::StringTransform;

class TopicRemapPlugin : public RosbagPlayPlugin {
 public:
  TopicRemapPlugin();

  void init(std::shared_ptr<rclcpp::Node>) override {}
  void add_options(CLI::App& app) override;
  void modify_playback(rosbag2_transport::PlayOptions& options) override;
  void on_start(rosbag2_cpp::Reader& reader, const rclcpp::Logger* logger = nullptr) override;
  void on_stop() override {}

 private:
  std::string prefix;
  std::vector<std::string> remaps;
  std::vector<std::string> bag_args;
};

TopicRemapPlugin::TopicRemapPlugin() {}

void TopicRemapPlugin::add_options(CLI::App& app) {
  app.add_option("-t,--topic-substitution", remaps)
      ->description("Apply remap to topics (match and substitution are separated by :)");
  app.add_option("--topic-prefix", prefix)->description("Remap topics to have new prefix");
}

using ArgVec = std::vector<std::string>;

void TopicRemapPlugin::modify_playback(rosbag2_transport::PlayOptions& options) {
  options.topic_remapping_options.insert(
      options.topic_remapping_options.end(), bag_args.begin(), bag_args.end());
}

void TopicRemapPlugin::on_start(rosbag2_cpp::Reader& reader, const rclcpp::Logger* logger) {
  std::vector<StringTransform> transforms;
  for (const auto& sub : remaps) {
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Substitute, sub, logger));
  }

  if (!prefix.empty()) {
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Prefix, prefix, logger));
  }

  const auto all_topics = reader.get_all_topics_and_types();

  for (const auto& data : all_topics) {
    auto topic = data.name;
    for (const auto& transform : transforms) {
      topic = transform.apply(topic);
    }

    if (topic == data.name) {
      continue;
    }

    bag_args.push_back("-r");
    bag_args.push_back(data.name + ":=" + topic);
    if (logger) {
      RCLCPP_INFO_STREAM(*logger, "Remapping '" << data.name << "' -> '" << topic << "'");
    }
  }
}

}  // namespace ianvs

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ianvs::TopicRemapPlugin, ianvs::RosbagPlayPlugin)
