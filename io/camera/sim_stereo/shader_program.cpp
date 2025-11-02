#include "shader_program.hpp"

#include <stdexcept>
#include <utility>

#include <glad/glad.h>

namespace fsai::io::camera::sim_stereo {
namespace {
unsigned int compileShader(unsigned int type, const std::string& source) {
  GLuint shader = glCreateShader(type);
  const char* src = source.c_str();
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  GLint compiled = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (compiled != GL_TRUE) {
    GLint log_length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
    std::string log(static_cast<std::size_t>(log_length), '\0');
    glGetShaderInfoLog(shader, log_length, nullptr, log.data());
    glDeleteShader(shader);
    throw std::runtime_error("Shader compile failed: " + log);
  }
  return shader;
}
}

ShaderProgram::ShaderProgram(std::string vertex_src, std::string fragment_src)
    : vertex_src_(std::move(vertex_src)), fragment_src_(std::move(fragment_src)) {}

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept { *this = std::move(other); }

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
  if (this != &other) {
    destroy();
    vertex_src_ = std::move(other.vertex_src_);
    fragment_src_ = std::move(other.fragment_src_);
    program_ = other.program_;
    uniform_cache_ = std::move(other.uniform_cache_);
    other.program_ = 0;
  }
  return *this;
}

ShaderProgram::~ShaderProgram() { destroy(); }

void ShaderProgram::destroy() {
  if (program_ != 0) {
    glDeleteProgram(program_);
    program_ = 0;
  }
  uniform_cache_.clear();
}

void ShaderProgram::setVertexSource(const std::string& src) {
  vertex_src_ = src;
  destroy();
}

void ShaderProgram::setFragmentSource(const std::string& src) {
  fragment_src_ = src;
  destroy();
}

bool ShaderProgram::compile() {
  if (program_ != 0) {
    return true;
  }
  if (vertex_src_.empty() || fragment_src_.empty()) {
    return false;
  }

  GLuint vs = compileShader(GL_VERTEX_SHADER, vertex_src_);
  GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragment_src_);

  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glLinkProgram(program_);

  GLint linked = GL_FALSE;
  glGetProgramiv(program_, GL_LINK_STATUS, &linked);
  glDeleteShader(vs);
  glDeleteShader(fs);

  if (linked != GL_TRUE) {
    GLint log_length = 0;
    glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &log_length);
    std::string log(static_cast<std::size_t>(log_length), '\0');
    glGetProgramInfoLog(program_, log_length, nullptr, log.data());
    glDeleteProgram(program_);
    program_ = 0;
    throw std::runtime_error("Program link failed: " + log);
  }

  uniform_cache_.clear();
  return true;
}

void ShaderProgram::use() const {
  if (program_ != 0) {
    glUseProgram(program_);
  }
}

int ShaderProgram::uniformLocation(const char* name) const {
  auto it = uniform_cache_.find(name);
  if (it != uniform_cache_.end()) {
    return it->second;
  }
  GLint location = glGetUniformLocation(program_, name);
  uniform_cache_.emplace(name, location);
  return location;
}

void ShaderProgram::setMat4(const char* name, const std::array<float, 16>& matrix) const {
  int location = uniformLocation(name);
  if (location >= 0) {
    glUniformMatrix4fv(location, 1, GL_FALSE, matrix.data());
  }
}

ShaderProgram buildDefaultConeShader() {
  static constexpr const char* kConeVs = R"(#version 330 core
layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_region;
layout(location = 2) in mat4 in_model;
layout(location = 6) in vec3 in_body_color;
layout(location = 7) in vec3 in_stripe_color;
layout(location = 8) in float in_stripe_count;

uniform mat4 u_view;
uniform mat4 u_proj;

out vec3 v_local_pos;
flat out float v_region;
flat out vec3 v_body_color;
flat out vec3 v_stripe_color;
flat out float v_stripe_count;

void main() {
  v_local_pos = in_position;
  v_region = in_region;
  v_body_color = in_body_color;
  v_stripe_color = in_stripe_color;
  v_stripe_count = in_stripe_count;
  gl_Position = u_proj * u_view * in_model * vec4(in_position, 1.0);
}
)";

  static constexpr const char* kConeFs = R"(#version 330 core
in vec3 v_local_pos;
flat in float v_region;
flat in vec3 v_body_color;
flat in vec3 v_stripe_color;
flat in float v_stripe_count;

out vec4 out_color;

const float kBaseColor = 0.18;
const float kStripeHalfWidth = 0.08;
const int kMaxStripes = 4;

void main() {
  if (v_region < 0.5) {
    out_color = vec4(kBaseColor, kBaseColor, kBaseColor, 1.0);
    return;
  }

  vec3 color = v_body_color;
  int stripes = int(round(clamp(v_stripe_count, 0.0, float(kMaxStripes))));
  if (stripes > 0) {
    float x_norm = clamp(v_local_pos.x + 0.5, 0.0, 1.0);
    for (int i = 0; i < kMaxStripes; ++i) {
      if (i >= stripes) {
        break;
      }
      float center = (float(i) + 1.0) / (float(stripes) + 1.0);
      float delta = abs(x_norm - center);
      if (delta > 0.5) {
        delta = 1.0 - delta;
      }
      if (delta < kStripeHalfWidth) {
        color = v_stripe_color;
        break;
      }
    }
  }

  out_color = vec4(color, 1.0);
}
)";

  ShaderProgram program;
  program.setVertexSource(kConeVs);
  program.setFragmentSource(kConeFs);
  program.compile();
  return program;
}

ShaderProgram buildDefaultGroundShader() {
  static constexpr const char* kGroundVs = R"(#version 330 core
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

  static constexpr const char* kGroundFs = R"(#version 330 core
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
