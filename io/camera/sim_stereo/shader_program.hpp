#pragma once

#include <array>
#include <string>
#include <unordered_map>

namespace fsai::io::camera::sim_stereo {

class ShaderProgram {
 public:
  ShaderProgram() = default;
  ShaderProgram(std::string vertex_src, std::string fragment_src);
  ShaderProgram(const ShaderProgram&) = delete;
  ShaderProgram& operator=(const ShaderProgram&) = delete;
  ShaderProgram(ShaderProgram&& other) noexcept;
  ShaderProgram& operator=(ShaderProgram&& other) noexcept;
  ~ShaderProgram();

  void setVertexSource(const std::string& src);
  void setFragmentSource(const std::string& src);

  bool compile();
  void use() const;
  void setMat4(const char* name, const std::array<float, 16>& matrix) const;

  unsigned int id() const { return program_; }

 private:
  int uniformLocation(const char* name) const;
  void destroy();

  std::string vertex_src_;
  std::string fragment_src_;
  unsigned int program_ = 0;
  mutable std::unordered_map<std::string, int> uniform_cache_;
};

ShaderProgram buildDefaultConeShader();
ShaderProgram buildDefaultGroundShader();

}  // namespace fsai::io::camera::sim_stereo
