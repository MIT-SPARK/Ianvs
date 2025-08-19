#include <filesystem>

#include <CLI/CLI.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_transport/player.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

#include "ianvs/app/rosbag_play_plugins.h"

using namespace std::chrono_literals;
using PluginVec = std::vector<std::shared_ptr<ianvs::RosbagPlayPlugin>>;

class BagWrapper : public rclcpp::Node {
 public:
  BagWrapper() : Node("bag_wrapper"), running(false) {}
  ~BagWrapper() { stop(); }

  void start(const std::filesystem::path& bag_path, const std::vector<std::string>& cmd_args);
  bool stop();

  bool running;

 private:
  std::unique_ptr<rosbag2_transport::Player> player_;
  rclcpp::TimerBase::SharedPtr timer_;
};

void BagWrapper::start(const std::filesystem::path& bag_path,
                       const std::vector<std::string>& cmd_args) {
  using namespace std::chrono_literals;
  running = true;

  auto node_opts = rclcpp::NodeOptions().use_intra_process_comms(true);
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = bag_path;
  rosbag2_transport::PlayOptions play_opts;

  player_.reset(
      new rosbag2_transport::Player(storage_opts, play_opts, "rosbag2_player", node_opts));
  player_->play();
  auto timer_callback = [this]() -> void {
    if (player_ && player_->wait_for_playback_to_finish(1ms)) {
      running = false;
      timer_->cancel();
    }
  };

  timer_ = this->create_wall_timer(10ms, timer_callback);
}

bool BagWrapper::stop() {
  if (player_) {
    player_->stop();
  }

  player_.reset();
  return true;
}

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

std::vector<std::string> get_bag_args(const std::string& bag_path,
                                      const std::vector<std::string>& remaining) {
  std::vector<std::string> cmd_args{"bag", "play", bag_path};
  for (const auto& arg : remaining) {
    if (arg == "--") {
      continue;
    }

    if (arg == "--ros-args") {
      break;
    }

    cmd_args.push_back(arg);
  }

  return cmd_args;
}

PluginVec load_plugins(pluginlib::ClassLoader<ianvs::RosbagPlayPlugin>& loader,
                       const rclcpp::Logger& logger) {
  PluginVec plugins;
  const auto classes = loader.getDeclaredClasses();

  std::stringstream ss;
  ss << "[";
  auto iter = classes.begin();
  while (iter != classes.end()) {
    ss << *iter;
    ++iter;
    if (iter != classes.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  RCLCPP_DEBUG_STREAM(logger, "Loading plugin classes: " << ss.str());

  for (const auto& to_load : classes) {
    try {
      plugins.push_back(loader.createSharedInstance(to_load));
    } catch (const pluginlib::PluginlibException& e) {
      RCLCPP_ERROR_STREAM(logger,
                          "Unable to load registered plugin '" << to_load << "': " << e.what());
    }
  }

  return plugins;
}

std::vector<std::string> process_bag(const std::filesystem::path& bag_path,
                                     const PluginVec& plugins,
                                     const std::vector<std::string>& remaining) {
  if (!std::filesystem::exists(bag_path)) {
    return {};
  }

  rosbag2_storage::StorageOptions opts;
  opts.uri = bag_path;
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
  if (!reader) {
    return {};
  }

  reader->open(opts);
  for (const auto& plugin : plugins) {
    plugin->on_start(*reader);
  }

  auto cmd_args = get_bag_args(bag_path, remaining);
  for (const auto& plugin : plugins) {
    cmd_args = plugin->modify_args(cmd_args);
  }

  return cmd_args;
}

void stop_plugins(const PluginVec& plugins) {
  for (const auto& plugin : plugins) {
    plugin->on_stop();
  }
}

void cleanup(PluginVec& plugins, rclcpp::Executor& executor) {
  if (rclcpp::ok()) {
    std::atomic<bool> finished = false;
    std::thread stop_thread([&]() {
      for (const auto& plugin : plugins) {
        plugin->on_stop();
      }
      finished = true;
    });

    while (!finished) {
      executor.spin_once(1000ns);
    }

    stop_thread.join();
  } else {
    for (const auto& plugin : plugins) {
      plugin->on_stop();
    }
  }

  plugins.clear();
  rclcpp::shutdown();
}

struct AppArgs {
  bool verbose = false;
  std::filesystem::path bag;
};

int main(int argc, char** argv) {
  const auto ros_argv = get_ros_args(argc, argv);
  rclcpp::init(ros_argv.size(), ros_argv.data());
  rclcpp::get_logger("rosbag2_storage").set_level(rclcpp::Logger::Level::Warn);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<BagWrapper>();
  executor.add_node(node);

  pluginlib::ClassLoader<ianvs::RosbagPlayPlugin> loader("ianvs", "ianvs::RosbagPlayPlugin");
  auto plugins = load_plugins(loader, node->get_logger());
  for (const auto& plugin : plugins) {
    plugin->init(node);
  }

  CLI::App app("Utility to play a rosbag after modfying and publishing transforms");
  // argv = app.ensure_utf8(argv); // TODO(nathan): re-enable this after 22.04 EOL
  app.allow_extras();

  // NOTE(nathan) multiple bags and -- separator don't work
  AppArgs args;
  app.add_option("bag", args.bag)->required()->description("primary bag to read static tfs from");
  app.add_flag("-v,--verbose", args.verbose, "show transform results");
  for (const auto& plugin : plugins) {
    plugin->add_options(app);
  }

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    cleanup(plugins, executor);
    return app.exit(e);
  }

  const auto cmd_args = process_bag(args.bag, plugins, app.remaining());
  if (cmd_args.empty()) {
    cleanup(plugins, executor);
    return 1;
  }

  node->start(args.bag, cmd_args);
  while (rclcpp::ok() && node->running) {
    executor.spin_once(1000ns);
  }

  node->stop();
  cleanup(plugins, executor);
}
