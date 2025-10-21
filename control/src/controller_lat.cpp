#include <stdexcept>

namespace fsai {
namespace control {

void ControllerLat_NotImplemented() {
    throw std::runtime_error("Lateral controller stage is not yet implemented");
}

}  // namespace control
}  // namespace fsai
