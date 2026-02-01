#include <yaml-cpp/yaml.h>

#include <filesystem>

#include <CLI/CLI.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/buffer_core.hpp>

using geometry_msgs::msg::TransformStamped;

struct AppArgs {
  std::filesystem::path tf_filepath;
  std::string from_frame;
  std::string to_frame;

  void add_to_app(CLI::App& app);
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("tf_filepath", tf_filepath)->required()->description("File to read");
  app.add_option("from_frame", from_frame)->required()->description("from in to_T_from");
  app.add_option("to_frame", to_frame)->required()->description("to in to_T_from");
}

void loadTransformsFromFile(const std::filesystem::path& filepath, tf2::BufferCore& buffer) {
  if (!std::filesystem::exists(filepath)) {
    std::cerr << "Invalid filepath: " << filepath << std::endl;
    return;
  }

  const auto node = YAML::LoadFile(filepath);
  for (const auto& tf : node["frames"]) {
    try {
      std::string frame_id = tf["frame_id"].as<std::string>();
      std::string child_id = tf["child_frame_id"].as<std::string>();

      TransformStamped msg;
      msg.header.frame_id = frame_id;
      msg.child_frame_id = child_id;
      msg.transform.translation.x = tf["x"].as<double>();
      msg.transform.translation.y = tf["y"].as<double>();
      msg.transform.translation.z = tf["z"].as<double>();
      msg.transform.rotation.w = tf["qw"].as<double>();
      msg.transform.rotation.x = tf["qx"].as<double>();
      msg.transform.rotation.y = tf["qy"].as<double>();
      msg.transform.rotation.z = tf["qz"].as<double>();
      buffer.setTransform(msg, "file", true);
    } catch (std::exception& e) {
      std::cerr << "Failed to parse " << tf << ": " << e.what() << std::endl;
      continue;
    }
  }
}

std::string getFrameList(const tf2::BufferCore& buffer) {
    const auto all_frames = buffer.getAllFrameNames();
    std::stringstream ss;
    for (const auto& frame: all_frames) {
      ss << " - " << frame << "\n";
    }

    return ss.str();
}

int main(int argc, char** argv) {
  CLI::App app("Utility that looks up transform between two frames from a file");
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_to_app(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  tf2::BufferCore buffer;
  loadTransformsFromFile(args.tf_filepath, buffer);
  std::string error;
  tf2::TimePoint stamp;
  if (!buffer.canTransform(args.to_frame, args.from_frame, stamp, &error)) {
    std::cerr << "Cannot find " << args.to_frame << "_T_" << args.from_frame << ": " << error
              << std::endl;

    std::cerr << "Available frames:\n" << getFrameList(buffer) << std::endl;
    return 1;
  }

  const auto tf = buffer.lookupTransform(args.to_frame, args.from_frame, stamp);
  return 0;
}
