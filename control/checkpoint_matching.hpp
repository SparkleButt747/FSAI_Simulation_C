#pragma once

#include <vector>
#include "Transform.h"
#include "Vector.h"

namespace fsai::control {

struct MatchedGate {
  Transform left;
  Transform right;
  Transform checkpoint;
};

std::vector<MatchedGate> MatchConesToGates(
    const std::vector<Transform>& left,
    const std::vector<Transform>& right);

}  // namespace fsai::control
