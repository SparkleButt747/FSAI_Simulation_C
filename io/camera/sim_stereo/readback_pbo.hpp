#pragma once

#include <cstddef>
#include <vector>
#include <cstdint>

namespace fsai::io::camera::sim_stereo {

class ReadbackPbo {
 public:
  ReadbackPbo();
  ReadbackPbo(const ReadbackPbo&) = delete;
  ReadbackPbo& operator=(const ReadbackPbo&) = delete;
  ReadbackPbo(ReadbackPbo&& other) noexcept;
  ReadbackPbo& operator=(ReadbackPbo&& other) noexcept;
  ~ReadbackPbo();

  void read(unsigned int fbo, int width, int height, std::vector<uint8_t>& out);

 private:
  void ensureCapacity(std::size_t bytes);

  unsigned int pbo_ = 0;
  std::size_t capacity_ = 0;
};

}  // namespace fsai::io::camera::sim_stereo
