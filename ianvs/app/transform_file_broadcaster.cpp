#include <yaml-cpp/yaml.h>

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

class TransformFileBroadcaster : public rclcpp::Node {
 public:
  TransformFileBroadcaster()
      : Node("transform_file_broadcaster"),
        broadcaster_(new tf2_ros::StaticTransformBroadcaster(this)) {
    declare_parameter("filepath", "");
    std::string filepath;
    get_parameter("filepath", filepath);
    if (filepath.empty()) {
      RCLCPP_WARN_STREAM(get_logger(), "No filepath provided!");
      broadcaster_.reset();
      return;
    }

    std::filesystem::path transform_file(filepath);
    if (!std::filesystem::exists(transform_file)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Provided file " << transform_file << " does not exist!");
      broadcaster_.reset();
      return;
    }

    const auto stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    try {
      const auto node = YAML::LoadFile(transform_file);
      for (const auto& tf : node["frames"]) {
        auto& msg = transforms.emplace_back();
        msg.header.stamp = stamp;
        msg.header.frame_id = tf["frame_id"].as<std::string>();
        msg.child_frame_id = tf["child_frame_id"].as<std::string>();
        msg.transform.translation.x = tf["x"].as<double>();
        msg.transform.translation.y = tf["y"].as<double>();
        msg.transform.translation.z = tf["z"].as<double>();
        msg.transform.rotation.w = tf["qw"].as<double>();
        msg.transform.rotation.x = tf["qx"].as<double>();
        msg.transform.rotation.y = tf["qy"].as<double>();
        msg.transform.rotation.z = tf["qz"].as<double>();
      }
    } catch (std::exception& e) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Provided file " << transform_file << " was invalid: " << e.what());
      broadcaster_.reset();
      return;
    }

    if (transforms.empty()) {
      RCLCPP_WARN_STREAM(get_logger(), "No transforms provided!");
      broadcaster_.reset();
      return;
    }

    broadcaster_->sendTransform(transforms);
  }

 private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformFileBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
