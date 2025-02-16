#pragma once
#include <memory>

#include <sensor_msgs/msg/image.hpp>

#include "ianvs/node_handle.h"

namespace ianvs {

class ImagePublisher {
 public:
  ImagePublisher();
  ImagePublisher(NodeHandle::NodeInterface nh,
                 const std::string& topic,
                 const rclcpp::QoS& qos = rclcpp::SensorDataQoS());
  ~ImagePublisher();

  void publish(const sensor_msgs::msg::Image& msg) const;

  void publish(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const;

  void publish(sensor_msgs::msg::Image::UniquePtr&& msg) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ianvs
