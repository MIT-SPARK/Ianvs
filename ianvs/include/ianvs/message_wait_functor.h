#pragma once
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include "ianvs/node_handle.h"

namespace ianvs {

template <typename MsgT>
struct MessageWaitFunctor {
  using PtrT = typename MsgT::ConstSharedPtr;
  void callback(const PtrT& msg) { msg_ = msg; }

  PtrT wait(NodeHandle nh, bool spin_required) {
    auto base = nh.node().get<rclcpp::node_interfaces::NodeBaseInterface>();

    rclcpp::WallRate r(10.0);
    while (rclcpp::ok() && !msg_) {
      r.sleep();
      if (spin_required) {
        rclcpp::spin_some(base);
      }
    }

    return msg_;
  }

 private:
  PtrT msg_;
};

template <typename MsgT>
std::optional<MsgT> getSingleMessage(NodeHandle nh,
                                     const std::string& topic,
                                     bool spin_required,
                                     const rclcpp::QoS& qos = rclcpp::QoS(1)) {
  MessageWaitFunctor<MsgT> functor;
  [[maybe_unused]] const auto sub = nh.create_subscription<MsgT>(
      topic, qos, &MessageWaitFunctor<MsgT>::callback, &functor);
  const auto msg = functor.wait(nh, spin_required);
  if (!msg) {
    return std::nullopt;
  }

  return *msg;
}

}  // namespace ianvs
