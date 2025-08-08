#include "ianvs/node_init.h"

namespace ianvs {

NodeGuard::~NodeGuard() {}

NodeGuard init(int& argc, char** argv, const std::string& node_name) { return {}; }

}  // namespace ianvs
