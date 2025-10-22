#include "io/camera/sim_stereo/shader_program.hpp"

#include <utility>

namespace fsai::io::camera::sim_stereo {

ShaderProgram::ShaderProgram(std::string vertex_src, std::string fragment_src)
    : vertex_src_(std::move(vertex_src)), fragment_src_(std::move(fragment_src)) {}

void ShaderProgram::setVertexSource(const std::string& src) {
  vertex_src_ = src;
  compiled_ = false;
}

void ShaderProgram::setFragmentSource(const std::string& src) {
  fragment_src_ = src;
  compiled_ = false;
}

bool ShaderProgram::compile() {
  compiled_ = !vertex_src_.empty() && !fragment_src_.empty();
  return compiled_;
}

ShaderProgram buildDefaultConeShader() {
  static constexpr const char* kConeVs = R"( #version 330 core
    layout(location = 0) in vec3 in_position;
    layout(location = 1) in vec3 in_color;
    layout(location = 2) in mat4 in_model;

    uniform mat4 u_view;
    uniform mat4 u_proj;

    out vec3 v_color;

    void main() {
      v_color = in_color;
      gl_Position = u_proj * u_view * in_model * vec4(in_position, 1.0);
    }
  )";

  static constexpr const char* kConeFs = R"( #version 330 core
    in vec3 v_color;
    out vec4 out_color;
    void main() {
      out_color = vec4(v_color, 1.0);
    }
  )";

  ShaderProgram program;
  program.setVertexSource(kConeVs);
  program.setFragmentSource(kConeFs);
  program.compile();
  return program;
}

ShaderProgram buildDefaultGroundShader() {
  static constexpr const char* kGroundVs = R"( #version 330 core
    layout(location = 0) in vec3 in_position;
    layout(location = 1) in vec3 in_color;

    uniform mat4 u_view;
    uniform mat4 u_proj;

    out vec3 v_color;

    void main() {
      v_color = in_color;
      gl_Position = u_proj * u_view * vec4(in_position, 1.0);
    }
  )";

  static constexpr const char* kGroundFs = R"( #version 330 core
    in vec3 v_color;
    out vec4 out_color;
    void main() {
      out_color = vec4(v_color, 1.0);
    }
  )";

  ShaderProgram program;
  program.setVertexSource(kGroundVs);
  program.setFragmentSource(kGroundFs);
  program.compile();
  return program;
}

}  // namespace fsai::io::camera::sim_stereo
