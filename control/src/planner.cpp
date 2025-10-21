#include <stdexcept>

namespace fsai {
namespace control {

void Planner_NotImplemented() {
    throw std::runtime_error("Planner pipeline stage is not yet implemented");
}

}  // namespace control
}  // namespace fsai
