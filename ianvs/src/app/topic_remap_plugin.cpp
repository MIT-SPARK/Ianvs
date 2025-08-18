
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
  std::vector<std::string> modify_args(const std::vector<std::string>& args) override;
  void add_options(CLI::App& app) override;
  void on_start(rosbag2_cpp::Reader& reader, const rclcpp::Logger* logger = nullptr) override;
  void on_stop() override {}

 private:
  std::vector<std::string> remaps;
  std::vector<std::string> bag_args;
};

TopicRemapPlugin::TopicRemapPlugin() {}

void TopicRemapPlugin::add_options(CLI::App& app) {
  app.add_option("-t,--topic-substitution", remaps)
      ->description("apply remap to topics (match and substituion are separated by :)");
}

using ArgVec = std::vector<std::string>;

ArgVec TopicRemapPlugin::modify_args(const ArgVec& args) {
  bool added_remap = false;
  std::vector<std::string> new_args;
  for (const auto& arg : args) {
    new_args.push_back(arg);
    if (arg == "--remap") {
      new_args.insert(new_args.end(), bag_args.begin(), bag_args.end());
      added_remap = true;
    }
  }

  if (!added_remap && !bag_args.empty()) {
    new_args.push_back("--remap");
    new_args.insert(new_args.end(), bag_args.begin(), bag_args.end());
  }

  return new_args;
}

void TopicRemapPlugin::on_start(rosbag2_cpp::Reader& reader, const rclcpp::Logger* logger) {
  std::vector<StringTransform> transforms;
  for (const auto& sub : remaps) {
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Substitute, sub, logger));
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

    bag_args.push_back(data.name + ":=" + topic);
    if (logger) {
      RCLCPP_INFO_STREAM(*logger, "Remapping '" << data.name << "' -> '" << topic << "'");
    }
  }
}

}  // namespace ianvs

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ianvs::TopicRemapPlugin, ianvs::RosbagPlayPlugin)
