#include <stdexcept>

namespace fsai {
namespace control {

void Scheduler_NotImplemented() {
    throw std::runtime_error("Command scheduler stage is not yet implemented");
}

}  // namespace control
}  // namespace fsai
