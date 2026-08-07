#pragma once

#include <string>

#include "ianvs/node_handle.h"
#include "ianvs/visibility_control.h"

namespace ianvs {

class CurrentNode {
 public:
  static void init(std::shared_ptr<rclcpp::Node> node);

  static rclcpp::Node* get();

  static void clear();

 private:
  static CurrentNode& instance();

  CurrentNode();

  std::shared_ptr<rclcpp::Node> node_;
  static std::unique_ptr<CurrentNode> s_instance_;
};

struct NodeGuard {
  NodeGuard() = default;
  ~NodeGuard();
};

[[nodiscard]] IANVS_PUBLIC NodeGuard init_node(int& argc,
                                               char** argv,
                                               const std::string& node_name);

}  // namespace ianvs
