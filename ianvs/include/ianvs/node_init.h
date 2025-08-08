#pragma once

#include <string>

#include "ianvs/node_handle.h"
#include "ianvs/visibility_control.h"

namespace ianvs {

class CurrentNode {
 public:
  static CurrentNode& instance();

 private:
  CurrentNode();

  std::shared_ptr<rclcpp::Node> node_;
};

struct NodeGuard {
  NodeGuard() = default;
  ~NodeGuard();
};

IANVS_PUBLIC [[nodiscard]] NodeGuard init(int& argc,
                                          char** argv,
                                          const std::string& node_name);

}  // namespace ianvs
