#pragma once

#include <string>
#include <vector>

namespace fsai::io::camera::sim_stereo {

// Lightweight shader wrapper that stores GLSL sources.  In the headless
// renderer we do not depend on a specific GL loader, so the compilation step is
// deferred until the runtime binds a context.  For the CPU based fallback used
// in the tests we simply keep the sources so higher level systems can be aware
// of what would have been uploaded.
class ShaderProgram {
 public:
  ShaderProgram() = default;
  ShaderProgram(std::string vertex_src, std::string fragment_src);

  void setVertexSource(const std::string& src);
  void setFragmentSource(const std::string& src);

  const std::string& vertexSource() const { return vertex_src_; }
  const std::string& fragmentSource() const { return fragment_src_; }

  // Pretend to compile the program.  In a full OpenGL build this would create
  // shader objects and link them.  Returning true signals that the sources are
  // non-empty and could be uploaded.
  bool compile();

  bool compiled() const { return compiled_; }

 private:
  std::string vertex_src_;
  std::string fragment_src_;
  bool compiled_ = false;
};

ShaderProgram buildDefaultConeShader();
ShaderProgram buildDefaultGroundShader();

}  // namespace fsai::io::camera::sim_stereo
