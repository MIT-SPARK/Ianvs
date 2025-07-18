#include "ianvs/node_handle.h"

namespace ianvs {

std::string join_namespace(const std::string& ns, const std::string& topic) {
  if (topic.empty()) {
    return ns;
  }

  if (ns.empty() || topic[0] == '~' || topic[0] == '/') {
    return topic;
  }

  return ns + (ns.back() == '/' ? "" : "/") + topic;
}

NodeHandle::NodeHandle(NodeInterface node, const std::string& ns)
    : node_(node), ns_(ns) {}

NodeHandle& NodeHandle::operator/=(const std::string& ns) {
  ns_ = join_namespace(ns_, ns);
  return *this;
}

std::string NodeHandle::resolve_name(const std::string& name, bool is_service) const {
  auto base = node_.get<rclcpp::node_interfaces::NodeBaseInterface>();
  return base->resolve_topic_or_service_name(join_namespace(ns_, name), is_service);
}

}  // namespace ianvs
