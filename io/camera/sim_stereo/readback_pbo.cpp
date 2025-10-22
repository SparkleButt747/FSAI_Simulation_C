#include "io/camera/sim_stereo/readback_pbo.hpp"

#include <algorithm>
#include <cstddef>

namespace fsai::io::camera::sim_stereo {

void ReadbackPbo::resize(size_t bytes) {
  if (staging_.size() < bytes) {
    staging_.resize(bytes);
  }
}

void ReadbackPbo::read(const EyeBuffer& eye, std::vector<uint8_t>& out) {
  size_t required = static_cast<size_t>(eye.width) * static_cast<size_t>(eye.height) * 4;
  resize(required);
  auto count = static_cast<std::ptrdiff_t>(required);
  std::copy(eye.rgba.begin(), eye.rgba.begin() + count, staging_.begin());
  out.assign(staging_.begin(), staging_.begin() + count);
}

}  // namespace fsai::io::camera::sim_stereo
