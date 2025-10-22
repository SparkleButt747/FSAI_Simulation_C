#pragma once

#include <vector>

#include "io/camera/sim_stereo/fbo_stereo.hpp"

namespace fsai::io::camera::sim_stereo {

// CPU implementation of a Pixel Buffer Object helper.  The class mimics the
// interface of an asynchronous readback but simply copies into an internal
// staging buffer before exposing the data to the caller.
class ReadbackPbo {
 public:
  ReadbackPbo() = default;

  void resize(size_t bytes);
  void read(const EyeBuffer& eye, std::vector<uint8_t>& out);

 private:
  std::vector<uint8_t> staging_;
};

}  // namespace fsai::io::camera::sim_stereo
