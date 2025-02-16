/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "ianvs/spin_functions.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>

namespace ianvs {

struct ShutdownMonitor {
  explicit ShutdownMonitor(NodeHandle nh)
      : should_exit(false),
        service(nh.create_service<std_srvs::srv::Empty>(
            "shutdown", &ShutdownMonitor::callback, this)) {}

  void callback(const std_srvs::srv::Empty::Request::SharedPtr&,
                std_srvs::srv::Empty::Response::SharedPtr) {
    should_exit = true;
  }

  bool should_exit;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
};

bool haveClock(NodeHandle nh) {
  auto graph = nh.node().get<rclcpp::node_interfaces::NodeGraphInterface>();
  return graph->count_publishers("/clock") > 0;
}

void spinWhileClockPresent(NodeHandle nh) {
  ShutdownMonitor functor(nh);

  rclcpp::WallRate r(50);
  auto base = nh.node().get<rclcpp::node_interfaces::NodeBaseInterface>();

  RCLCPP_INFO(nh.logger(), "Waiting for bag to start");
  while (rclcpp::ok() && !haveClock(nh)) {
    rclcpp::spin_some(base);
    r.sleep();
  }

  RCLCPP_INFO(nh.logger(), "Running...");
  while (rclcpp::ok() && haveClock(nh) && !functor.should_exit) {
    rclcpp::spin_some(base);
    r.sleep();
  }

  rclcpp::spin_some(base);  // make sure all the callbacks are processed
  RCLCPP_INFO(nh.logger(), "Exiting!");
}

void spinUntilExitRequested(NodeHandle nh) {
  ShutdownMonitor functor(nh);

  rclcpp::WallRate r(50);
  auto base = nh.node().get<rclcpp::node_interfaces::NodeBaseInterface>();

  RCLCPP_INFO(nh.logger(), "Running...");
  while (rclcpp::ok() && !functor.should_exit) {
    rclcpp::spin_some(base);
    r.sleep();
  }

  rclcpp::spin_some(base);
  RCLCPP_INFO(nh.logger(), "Exiting!");
}

void spinAndWait(NodeHandle nh, bool exit_after_clock) {
  if (exit_after_clock) {
    spinWhileClockPresent(nh);
  } else {
    spinUntilExitRequested(nh);
  }
}

}  // namespace ianvs
