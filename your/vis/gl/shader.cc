#include "onboard/vis/gl/shader.h"

#include "glog/logging.h"

namespace qcraft {
namespace vis {

Shader::Shader(const char *vert_shader_src, const char *frag_shader_src) {
  LOG(INFO) << "Building shaders...";
  VLOG(1) << "Vertex shader source: " << std::endl
          << vert_shader_src << std::endl
          << "Fragment shader source: " << std::endl
          << frag_shader_src;

  GLuint vs, fs;
  GLint success;
  char info[1024];

  VLOG(1) << "Compiling vertex shader...";
  vs = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vs, 1, &vert_shader_src, nullptr);
  glCompileShader(vs);
  glGetShaderiv(vs, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vs, 1024, nullptr, info);
    LOG(ERROR) << "Vertex shader compile error: " << std::endl
               << info << std::endl;
    return;
  }

  VLOG(1) << "Compiling vertex shader...";
  fs = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fs, 1, &frag_shader_src, nullptr);
  glCompileShader(fs);
  glGetShaderiv(fs, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fs, 1024, nullptr, info);
    LOG(ERROR) << "Fragment shader compile error: " << std::endl
               << info << std::endl;
    return;
  }

  VLOG(1) << "Linking shaders...";
  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glLinkProgram(program_);
  glGetProgramiv(program_, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(program_, 1024, nullptr, info);
    LOG(ERROR) << "Shader program link error: " << std::endl
               << info << std::endl;
    return;
  }

  glDeleteShader(vs);
  glDeleteShader(fs);

  LOG(INFO) << "Shader building successful.";
}

Shader::Shader(const std::string &vert_shader_src,
               const std::string &frag_shader_src)
    : Shader(vert_shader_src.c_str(), frag_shader_src.c_str()) {}

Shader::~Shader() { glDeleteProgram(program_); }

GLint Shader::GetUniformLocation(const std::string &name) const {
  /*
    auto entry = uniform_locations_.find(name);
    if (entry == uniform_locations_.end()) {
      const GLint location = glGetUniformLocation(program_, name.c_str());
      auto result = uniform_locations_.insert(std::make_pair(name, location));
      CHECK(result.second);
      entry = result.first;
    }
    const GLint location = entry->second;
  */
  const GLint location = glGetUniformLocation(program_, name.c_str());
  if (location < 0) {
    LOG(WARNING) << "Uniform " << name << " not found.";
  }
  return location;
}

void Shader::SetUniformInt(const std::string &name, int value) const {
  glUniform1i(GetUniformLocation(name), value);
}

void Shader::SetUniformFloat(const std::string &name, float value) const {
  glUniform1f(GetUniformLocation(name), value);
}

void Shader::SetUniformVec4d(const std::string &name,
                             const Vec4d &value) const {
  glUniform4f(GetUniformLocation(name), value.x(), value.y(), value.z(),
              value.w());
}

void Shader::SetUniformVec3d(const std::string &name,
                             const Vec3d &value) const {
  glUniform3f(GetUniformLocation(name), value.x(), value.y(), value.z());
}

void Shader::SetUniformMat4d(const std::string &name,
                             const Mat4d &value) const {
  // Eigen matrix default order is column major, so is glUniformMatrix. But
  // the data type needs a conversion (double -> float).
  GLfloat data[16];
  for (int i = 0; i < 16; ++i) data[i] = value.data()[i];
  glUniformMatrix4fv(GetUniformLocation(name), 1, false, data);
}

void Shader::SetUniformMat3d(const std::string &name,
                             const Mat3d &value) const {
  // Eigen matrix default order is column major, so is glUniformMatrix. But
  // the data type needs a conversion (double -> float).
  GLfloat data[9];
  for (int i = 0; i < 9; ++i) data[i] = value.data()[i];
  glUniformMatrix3fv(GetUniformLocation(name), 1, false, data);
}

}  // namespace vis
}  // namespace qcraft
