#include <yaml-cpp/yaml.h>

#include <filesystem>

#include <CLI/CLI.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using geometry_msgs::msg::TransformStamped;

struct AppArgs {
  std::filesystem::path tf_filepath;
  std::string parent_frame;
  std::string child_frame;

  void add_to_app(CLI::App& app);
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("tf_filepath", tf_filepath)->required()->description("File to read");
  app.add_option("parent_frame", parent_frame)->description("Parent frame to lookup");
  app.add_option("child_frame", child_frame)->description("Child frame to lookup");
}

std::vector<TransformStamped> loadTransformsFromFile(const std::filesystem::path& filepath) {
  std::vector<TransformStamped> transforms;

  try {
    const auto node = YAML::LoadFile(filepath);
    for (const auto& tf : node["frames"]) {
      std::string frame_id = tf["frame_id"].as<std::string>();
      std::string child_id = tf["child_frame_id"].as<std::string>();

      auto& msg = transforms.emplace_back();
      msg.header.frame_id = frame_id;
      msg.child_frame_id = child_id;
      msg.transform.translation.x = tf["x"].as<double>();
      msg.transform.translation.y = tf["y"].as<double>();
      msg.transform.translation.z = tf["z"].as<double>();
      msg.transform.rotation.w = tf["qw"].as<double>();
      msg.transform.rotation.x = tf["qx"].as<double>();
      msg.transform.rotation.y = tf["qy"].as<double>();
      msg.transform.rotation.z = tf["qz"].as<double>();
    }
  } catch (std::exception& e) {
    return {};
  }

  return transforms;
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

  const auto transforms = loadTransformsFromFile(args.tf_filepath);
  // TODO(nathan) lookup tf

  return 0;
}
