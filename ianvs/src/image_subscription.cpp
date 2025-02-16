#include "ianvs/image_subscription.h"

namespace ianvs {

using sensor_msgs::msg::Image;

struct ImageSubscription::Impl {
  Impl(NodeHandle::NodeInterface nh,
       const std::string& topic,
       const rclcpp::QoS& qos,
       std::function<void(const Image::ConstSharedPtr&)>&& callback)
      : sub(rclcpp::create_subscription<Image>(nh, topic, qos, callback)) {}

  rclcpp::Subscription<Image>::SharedPtr sub;
};

ImageSubscription::ImageSubscription() = default;

ImageSubscription::ImageSubscription(NodeHandle::NodeInterface nh,
                                     const std::string& topic,
                                     const rclcpp::QoS& qos)
    : impl_(std::make_unique<Impl>(
          nh, topic, qos, [this](const Image::ConstSharedPtr& msg) {
            signalMessage(msg);
          })) {}

ImageSubscription::~ImageSubscription() = default;

}  // namespace ianvs
