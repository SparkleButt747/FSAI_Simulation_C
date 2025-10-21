#include <stdexcept>

namespace fsai {
namespace control {

void ControllerLon_NotImplemented() {
    throw std::runtime_error("Longitudinal controller stage is not yet implemented");
}

}  // namespace control
}  // namespace fsai
