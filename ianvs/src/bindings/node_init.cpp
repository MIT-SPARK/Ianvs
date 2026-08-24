#include "ianvs/bindings/node_init.h"

#include <string>

#include "ianvs/node_init.h"

namespace py = pybind11;

namespace ianvs::bindings::node_init {

[[nodiscard]] NodeGuard start_node(const std::string& name) {
  int argc = 1;
  auto temp = name;
  char* argv[] = {temp.data()};
  return init_node(argc, argv, name);
}

struct NodeInitHolder {
  explicit NodeInitHolder(const std::string& name);

  NodeGuard guard;
  const std::string name;
};

NodeInitHolder::NodeInitHolder(const std::string& _name) : guard(start_node(_name)), name(_name) {}

void add(py::module_& m) {
  py::class_<NodeInitHolder>(m, "_NodeInitHolder")
      .def(py::init<std::string>())
      .def_readonly("name", &NodeInitHolder::name);
}

}  // namespace ianvs::bindings::node_init
