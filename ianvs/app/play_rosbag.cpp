#include <filesystem>

#include <CLI/CLI.hpp>
#include <boost/process.hpp>
#include <boost/process/args.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

#include "ianvs/app/rosbag_play_plugins.h"

using namespace std::chrono_literals;

namespace bp = boost::process;

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
  BagWrapper() : Node("bag_wrapper") {}

  void start(const std::vector<std::string>& cmd_args) {
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
};

std::vector<char*> get_ros_args(int argc, char** argv) {
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

  return ros_argv;
}

int main(int argc, char** argv) {
  const auto ros_argv = get_ros_args(argc, argv);
  rclcpp::init(ros_argv.size(), ros_argv.data());
  rclcpp::get_logger("rosbag2_storage").set_level(rclcpp::Logger::Level::Warn);

  auto node = std::make_shared<BagWrapper>();

  CLI::App app("Utility to play a rosbag after modfying and publishing transforms");
  // TODO(when 22.04 support ends): re-enable this once all systems use CLI11 >= 2.3.2
  // argv = app.ensure_utf8(argv);
  app.allow_extras();

  // NOTE(nathan) for whatever reason, this doesn't get handled correctly when we use
  // the -- separator and multiple bags, so I give up
  std::filesystem::path bag;
  app.add_option("bag_path", bag)
      ->required()
      ->description("primary bag to read static tfs from");

  bool verbose = false;
  app.add_flag("-v,--verbose", verbose, "show transform results");

  std::vector<std::shared_ptr<ianvs::RosbagPlayPlugin>> plugins;
  pluginlib::ClassLoader<ianvs::RosbagPlayPlugin> loader("ianvs",
                                                         "ianvs::RosbagPlayPlugin");
  const auto classes = loader.getDeclaredClasses();
  for (const auto& to_load : classes) {
    try {
      plugins.push_back(loader.createSharedInstance(to_load));
    } catch (const pluginlib::PluginlibException& e) {
      RCLCPP_ERROR_STREAM(
          node->get_logger(),
          "Unable to load registered plugin '" << to_load << "': " << e.what());
    }
  }

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  const auto cmd_args = get_bag_args(bag, app.remaining(), {});
  node->start(cmd_args);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return node->stop();
}
