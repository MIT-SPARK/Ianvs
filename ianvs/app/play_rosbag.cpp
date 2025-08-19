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
  void add_to_app(CLI::App& app);

  std::filesystem::path bag;
  const rosbag2_transport::PlayOptions& play_options() const { return play; }

 private:
  rosbag2_transport::PlayOptions play;

  double play_delay_s = 0.0;
  double play_duration_s = 0.0;
  double play_start_offset_s = 0.0;

  std::filesystem::path qos_overrides_path;
  std::filesystem::path storage_config_path;
};

void AppArgs::add_to_app(CLI::App& app) {
  using rosbag2_transport::ServiceRequestsSource;
  const std::map<std::string, ServiceRequestsSource> mode_mapping{
      {"service_introspection", ServiceRequestsSource::SERVICE_INTROSPECTION},
      {"client_introspection", ServiceRequestsSource::CLIENT_INTROSPECTION},
  };

  app.add_option("bag_path", bag)->required()->description("Bag to open");

  app.add_option("--read-ahead-queue-size", play.read_ahead_queue_size)
      ->check(CLI::PositiveNumber)
      ->description("Size of message queue rosbag uses");
  app.add_option("-r,--rate", play.rate)
      ->check(CLI::PositiveNumber)
      ->description("Rate at which to play back messages");

  app.add_option("--topics", play.topics_to_filter)
      ->description("Space delimited list of topics to play");
  app.add_option("--services", play.services_to_filter)
      ->description("Space delimited list of services to play");
  app.add_option("-e,--regex", play.regex_to_filter)
      ->description("Play only topics and services matching regular expression");
  app.add_option("-x,--exclude-regex", play.exclude_regex_to_filter)
      ->description("Exclude topics and services matching regular expression");
  app.add_option("--exclude-topics", play.exclude_topics_to_filter)
      ->description("Space delimited list of topics not to play");
  app.add_option("--exclude-services", play.exclude_services_to_filter)
      ->description("Space delimited list of services not to play");

  app.add_option("--qos-profile-overrides-path", qos_overrides_path)
      ->check(CLI::ExistingFile)
      ->description("Path to a yaml file defining QoS profile overrides");

  app.add_flag("-l,--loop", play.loop, "Enables loop playback when playing a bagfile");

  app.add_option("-m,--remap", play.topic_remapping_options)
      ->description("List of topics to be remapped in the form 'old:=new'");

  app.add_option("--storage-config-file", storage_config_path)
      ->check(CLI::ExistingFile)
      ->description("Path to a yaml file defining storage specific configurations");

  const auto clock_opt = app.add_option("--clock", play.clock_publish_frequency)
                             ->expected(0, 1)
                             ->default_val(40.0)
                             ->description("Publish to '/clock' to act as a ROS time source");
  app.add_option("--clock-topics", play.clock_trigger_topics)
      ->description("Space delimited topics that will trigger a '/clock' update");
  app.add_flag("--clock-topics-all",
               play.clock_publish_on_topic_publish,
               "Publishes an update on '/clock' immediately before each replayed message");

  app.add_option("-d,--delay", play_delay_s)
      ->check(CLI::PositiveNumber)
      ->description("Sleep duration before play in seconds");
  app.add_option("--playback-duration", play_duration_s)
      ->check(CLI::PositiveNumber)
      ->description("Playback duration, in seconds");
  app.add_option("--playback-until-nsec", play.playback_until_timestamp)
      ->description("Playback until timestamp, expressed in nanoseconds since epoch");

  app.add_flag("--disable-keyboard-controls", play.disable_keyboard_controls, "Disables controls");
  app.add_flag("-p,--start-paused", play.start_paused, "Start in a paused state");
  app.add_option("--start-offset", play_start_offset_s)
      ->description("Start the playback player this many seconds into the bag file");

  // TODO(nathan) this might be milliseconds on the CLI side
  app.add_option("--wait-for-all-acked", play.wait_acked_timeout)
      ->description("Timeout for subscription acknlowedgement");
  app.add_flag("--disable-loan-message", play.disable_loan_message, "Disable loaned messages");
  app.add_flag("--publish-service-requests",
               play.publish_service_requests,
               "Publish recorded service requests instead of recorded service events");
  app.add_option("--service-requests-source", play.service_requests_source)
      ->check(CLI::Transformer(mode_mapping))
      ->description("Set the source of the service requests to be replayed");

  app.final_callback([this, clock_opt]() {
    if (!clock_opt->count()) {
      play.clock_publish_frequency = 0.0;
    }

    play.delay = rclcpp::Duration::from_seconds(play_delay_s);
    play.playback_duration = rclcpp::Duration::from_seconds(play_duration_s);
    play.start_offset = rclcpp::Duration::from_seconds(play_start_offset_s).nanoseconds();
  });
}

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
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_to_app(app);

  auto plugin_app = app.add_option_group("Plugins", "Command line arguments for player plugins");
  for (const auto& plugin : plugins) {
    plugin->add_options(*plugin_app);
  }

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    cleanup(plugins, executor);
    return app.exit(e);
  }

  const auto play_options = args.play_options();
  YAML::Node play_node;
  play_node["play_options"] = play_options;
  std::cerr << play_node << std::endl;

  std::vector<std::string> cmd_args;
  // const auto cmd_args = process_bag(args.bag, plugins, app.remaining());
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
