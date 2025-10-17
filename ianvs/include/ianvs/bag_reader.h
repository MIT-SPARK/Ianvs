#pragma once
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

namespace ianvs {

struct MessageInfo {
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> contents;
  const rosbag2_storage::TopicMetadata* metadata = nullptr;
  std::string topic() const { return contents ? contents->topic_name : ""; }
  std::string type() const { return metadata ? metadata->type : ""; }
  operator bool() const { return contents != nullptr && metadata != nullptr; }
  rclcpp::SerializedMessage serialized() const {
    return rclcpp::SerializedMessage(*contents->serialized_data);
  }

  template <typename T>
  typename T::Ptr as() const {
    if (!metadata || !contents) {
      return nullptr;
    }

    const auto name = rosidl_generator_traits::name<T>();
    if (metadata->type != name) {
      return nullptr;
    }

    auto msg = std::make_shared<T>();
    const rclcpp::Serialization<T> serialization;
    const auto serialized_contents = serialized();
    serialization.deserialize_message(&serialized_contents, msg.get());
    return msg;
  }
};

struct BagReader {
  BagReader(const std::filesystem::path& bagpath);
  MessageInfo next() const;
  operator bool() const { return reader != nullptr; }

  std::unique_ptr<rosbag2_cpp::Reader> reader;
  std::map<std::string, rosbag2_storage::TopicMetadata> lookup;
};

}  // namespace ianvs
