#pragma once
#include <message_filters/simple_filter.h>

#include <memory>

#include <sensor_msgs/msg/image.hpp>

#include "ianvs/node_handle.h"

namespace ianvs {

class ImageSubscription
    : public message_filters::SimpleFilter<sensor_msgs::msg::Image> {
 public:
  ImageSubscription() = default;
  ImageSubscription(NodeHandle::NodeInterface nh,
                    const std::string& topic,
                    const rclcpp::QoS& qos = rclcpp::SensorDataQoS());
  ~ImageSubscription();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ianvs
