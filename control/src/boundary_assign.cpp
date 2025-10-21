#include <stdexcept>

namespace fsai {
namespace control {

void BoundaryAssign_NotImplemented() {
    throw std::runtime_error("Boundary assignment pipeline stage is not yet implemented");
}

}  // namespace control
}  // namespace fsai
