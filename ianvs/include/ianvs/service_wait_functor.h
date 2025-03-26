#pragma once
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include "ianvs/node_handle.h"

namespace ianvs {

template <typename T>
std::future_status wait_for_result(
    T& future,
    std::chrono::seconds timeout,
    std::chrono::milliseconds spin_period = std::chrono::milliseconds(100)) {
  auto status = std::future_status::timeout;
  const auto start = std::chrono::steady_clock::now();
  while (status != std::future_status::ready && rclcpp::ok()) {
    if (std::chrono::steady_clock::now() - start > timeout) {
      break;
    }

    status = future.wait_for(spin_period);
  }
}

// TODO(nathan) service qos?

template <typename SrvT>
typename SrvT::Response::SharedPtr callService(
    NodeHandle nh,
    const std::string& service,
    std::unique_ptr<typename SrvT::Request>&& req,
    bool spin_required = false) {
  using namespace std::chrono_literals;
  auto base = nh.node().get<rclcpp::node_interfaces::NodeBaseInterface>();

  auto client = nh.create_client<SrvT>(service);
  while (!client->wait_for_service(10ms) && rclcpp::ok()) {
    if (spin_required) {
      rclcpp::spin_some(base);
    }
  }

  auto rep = client->async_send_request(std::move(req)).future.share();
  if (wait_for_result(rep, 1s) != std::future_status::ready) {
    return nullptr;
  }

  return rep.get();
}

}  // namespace ianvs
