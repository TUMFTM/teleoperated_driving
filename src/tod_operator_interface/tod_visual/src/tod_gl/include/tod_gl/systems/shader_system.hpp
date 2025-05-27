/**
 * @file shader_system.hpp
 * @brief Management of the shaders defined in the resources folder
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <string>

#include "glad/glad.h"
#include "glm/glm.hpp"

namespace tod_gl {

class ShaderSystem {
  public:
    ~ShaderSystem() = default;
    static unsigned int create_shader_program(const char *vertexPath, const char *fragmentPath);
    static void use_shader_program(const unsigned int programID);
    static void attach_shader_program(const unsigned int programID, const char *shaderPath, GLenum shaderType);
    static void set_shader_program_bool(const unsigned int programID, const std::string &name, bool value);
    static void set_shader_program_int(const unsigned int programID, const std::string &name, int value);
    static void set_shader_program_float(const unsigned int programID, const std::string &name, float value);
    static void set_shader_program_vec3(const unsigned int programID, const std::string &name, const glm::vec3 &value);
    static void set_shader_program_mat4(const unsigned int programID, const std::string &name, const glm::mat4 &value);

  private:
    ShaderSystem();
    static void check_shader_compile_errors(unsigned int programID, std::string type);
};

} // namespace tod_gl