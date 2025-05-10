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
using namespace std::chrono_literals;

namespace bp = boost::process;

struct StringTransform {
  enum class Type { Substitute, Filter, Prefix };

  StringTransform(const std::string& expr, const std::string& sub)
      : expr(std::regex(expr)), sub(sub) {}

  static StringTransform from_arg(Type type,
                                  const std::string& arg,
                                  const rclcpp::Logger& logger) {
    switch (type) {
      case Type::Substitute: {
        const auto pos = arg.find(":");
        if (pos == std::string::npos) {
          RCLCPP_WARN_STREAM(logger, "Invalid substitution: '" << arg << "'");
          return StringTransform("", "");
        }

        const auto expr = arg.substr(0, pos);
        const auto sub = arg.substr(pos + 1);
        return StringTransform(expr, sub);
      }
      case Type::Filter:
        return StringTransform(arg, "");
      case Type::Prefix:
        return StringTransform("^.+", arg + "$&");
      default:
        return StringTransform("", "");
    }
  }

  std::string apply(const std::string& frame_id) const {
    std::regex re(expr);
    return std::regex_replace(frame_id, re, sub);
  }

  const std::regex expr;
  const std::string sub;
};

struct FrameTransforms {
  struct Config {
    std::string prefix;
    std::string filter;
    std::vector<std::string> substitutions;
  };

  explicit FrameTransforms(const std::vector<StringTransform>& transforms)
      : transforms(transforms) {}

  static FrameTransforms fromConfig(const Config& config,
                                    const rclcpp::Logger& logger) {
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

std::string frame_name(const std::string& dest, const std::string& src) {
  return "'" + dest + "_T_" + src + "'";
}

std::vector<Transform> read_static_tfs(const std::filesystem::path& bag_path,
                                       const FrameTransforms& id_transforms,
                                       const rclcpp::Logger* logger = nullptr) {
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
      const auto parent_id = id_transforms.apply(tf.header.frame_id);
      const auto child_id = id_transforms.apply(tf.child_frame_id);
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

  return poses;
}

std::vector<std::string> get_remaps(const std::filesystem::path& bag_path,
                                    const std::vector<StringTransform>& transforms,
                                    const rclcpp::Logger* logger = nullptr) {
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

  reader->open(storage_options);
  const auto all_topics = reader->get_all_topics_and_types();

  std::vector<std::string> remaps;
  for (const auto& topic_data : all_topics) {
    auto new_topic = topic_data.name;
    for (const auto& transform : transforms) {
      new_topic = transform.apply(new_topic);
    }

    if (new_topic == topic_data.name) {
      continue;
    }

    remaps.push_back(topic_data.name + ":=" + new_topic);
    if (logger) {
      RCLCPP_INFO_STREAM(
          *logger, "Remapping '" << topic_data.name << "' -> '" << new_topic << "'");
    }
  }

  return remaps;
}

std::vector<std::string> get_bag_args(const std::string& bag_path,
                                      const std::vector<std::string>& remaining,
                                      const std::vector<std::string>& remaps) {
  std::vector<std::string> cmd_args{"bag", "play", bag_path};
  bool added_exclude = false;
  bool added_remap = false;
  for (const auto& arg : remaining) {
    if (arg == "--") {
      continue;
    }

    if (arg == "--ros-args") {
      break;
    }

    cmd_args.push_back(arg);
    if (arg == "--exclude-topics") {
      cmd_args.push_back("/tf_static");
      added_exclude = true;
    }

    if (arg == "--remap") {
      cmd_args.insert(cmd_args.end(), remaps.begin(), remaps.end());
      added_remap = true;
    }
  }

  if (!added_exclude) {
    cmd_args.push_back("--exclude-topics");
    cmd_args.push_back("/tf_static");
  }

  if (!added_remap && !remaps.empty()) {
    cmd_args.push_back("--remap");
    cmd_args.insert(cmd_args.end(), remaps.begin(), remaps.end());
  }

  return cmd_args;
}

class BagWrapper : public rclcpp::Node {
 public:
  BagWrapper(const FrameTransforms::Config& config,
             const std::string& bag_path,
             const std::vector<std::string>& bag_args,
             const std::vector<std::string>& topic_substitutions,
             bool verbose)
      : Node("tf_republisher"), broadcaster_(this) {
    const auto logger = get_logger();
    const auto frame_transforms = FrameTransforms::fromConfig(config, logger);
    const auto transforms =
        read_static_tfs(bag_path, frame_transforms, verbose ? &logger : nullptr);
    if (!transforms.empty()) {
      broadcaster_.sendTransform(transforms);
    }

    std::vector<StringTransform> topic_transforms;
    for (const auto& sub : topic_substitutions) {
      topic_transforms.push_back(
          StringTransform::from_arg(StringTransform::Type::Substitute, sub, logger));
    }

    const auto remaps =
        get_remaps(bag_path, topic_transforms, verbose ? &logger : nullptr);

    const auto cmd_args = get_bag_args(bag_path, bag_args, remaps);
    child_ = std::make_unique<bp::child>(bp::search_path("ros2"), bp::args(cmd_args));

    auto timer_callback = [this]() -> void {
      if (child_ && !child_->running()) {
        rclcpp::shutdown();
      }
    };

    timer_ = this->create_wall_timer(10ms, timer_callback);
  }

  int stop() {
    int exit_code = 0;
    if (child_) {
      child_->wait();
      exit_code = child_->exit_code();
    }

    child_.reset();
    return exit_code;
  }

  ~BagWrapper() { stop(); }

 private:
  std::unique_ptr<bp::child> child_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::StaticTransformBroadcaster broadcaster_;
};

int main(int argc, char** argv) {
  CLI::App app("Utility to play a rosbag after modfying and publishing transforms");
  argv = app.ensure_utf8(argv);
  app.allow_extras();

  // NOTE(nathan) for whatever reason, this doesn't get handled correctly when we use
  // the -- separator and multiple bags, so I give up
  std::filesystem::path bag;
  app.add_option("bag_path", bag)
      ->required()
      ->check(CLI::ExistingPath)
      ->description("primary bag to read static tfs from");

  bool verbose = false;
  app.add_flag("-v,--verbose", verbose, "show transform results");

  FrameTransforms::Config config;
  std::vector<std::string> remaps;
  app.add_option("-p,--prefix", config.prefix)
      ->take_last()
      ->description("prefix to apply to ALL frames");
  app.add_option("-f,--filter", config.filter)
      ->join('|')
      ->description("optional regex filter to drop frames (applied before prefix)");
  app.add_option("-s,--substitution", config.substitutions)
      ->description("apply substitution (match and substituion are separated by :)");
  app.add_option("-t,--topic-substitution", remaps)
      ->description("apply remap to topics (match and substituion are separated by :)");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  // filters argc and argv to only have wrapper node args
  std::vector<char*> ros_argv;
  ros_argv.push_back(argv[0]);

  bool found_ros_args = false;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--ros-args") {
      found_ros_args = true;
    }

    if (found_ros_args) {
      ros_argv.push_back(argv[i]);
    }
  }

  rclcpp::init(ros_argv.size(), ros_argv.data());

  auto node =
      std::make_shared<BagWrapper>(config, bag, app.remaining(), remaps, verbose);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return node->stop();
}
