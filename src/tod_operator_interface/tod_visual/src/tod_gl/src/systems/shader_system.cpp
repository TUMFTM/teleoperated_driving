/**
 * @file shader_system.cpp
 * @brief Management of the glsl shaders defined in the resources folder
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/systems/shader_system.hpp"

#include <fstream>
#include <iostream>

#include "tod_gl/scene/components.hpp"

namespace tod_gl {

unsigned int ShaderSystem::create_shader_program(const char *vertexPath, const char *fragmentPath) {
    const unsigned int programID = glCreateProgram();
    attach_shader_program(programID, vertexPath, GL_VERTEX_SHADER);
    attach_shader_program(programID, fragmentPath, GL_FRAGMENT_SHADER);
    glLinkProgram(programID);
    check_shader_compile_errors(programID, "PROGRAM");
    return programID;
}

void ShaderSystem::use_shader_program(const unsigned int programID) {
    glUseProgram(programID);
}

void ShaderSystem::attach_shader_program(const unsigned int programID, const char *shaderPath, GLenum shaderType) {
    // read source code from file
    std::ifstream shaderFile;
    shaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    std::string shaderCode;
    try {
        shaderFile.open(shaderPath);
        std::stringstream shaderStream;
        shaderStream << shaderFile.rdbuf();
        shaderFile.close();
        shaderCode = shaderStream.str();
    } catch (std::ifstream::failure e) {
        std::cout << "ERROR: could not read shader" << std::endl;
    }
    const char *source = shaderCode.c_str();

    // create, compile, attach and delete shader
    unsigned int shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    check_shader_compile_errors(shader, "SHADER");
    glAttachShader(programID, shader);
    glDeleteShader(shader);
}

void ShaderSystem::set_shader_program_bool(const unsigned int programID, const std::string &name, bool value) {
    ShaderSystem::use_shader_program(programID);
    glUniform1i(glGetUniformLocation(programID, name.c_str()), (int)value);
}

void ShaderSystem::set_shader_program_int(const unsigned int programID, const std::string &name, int value) {
    ShaderSystem::use_shader_program(programID);
    glUniform1i(glGetUniformLocation(programID, name.c_str()), value);
}

void ShaderSystem::set_shader_program_float(const unsigned int programID, const std::string &name, float value) {
    ShaderSystem::use_shader_program(programID);
    glUniform1f(glGetUniformLocation(programID, name.c_str()), value);
}

void ShaderSystem::set_shader_program_vec3(const unsigned int programID, const std::string &name, const glm::vec3 &value) {
    ShaderSystem::use_shader_program(programID);
    glUniform3f(glGetUniformLocation(programID, name.c_str()), value.x, value.y, value.z);
}

void ShaderSystem::set_shader_program_mat4(const unsigned int programID, const std::string &name, const glm::mat4 &value) {
    ShaderSystem::use_shader_program(programID);
    glUniformMatrix4fv(glGetUniformLocation(programID, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}

void ShaderSystem::check_shader_compile_errors(unsigned int programID, std::string type) {
    int success;
    char infoLog[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(programID, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(programID, 1024, NULL, infoLog);
            std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                      << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    } else {
        glGetProgramiv(programID, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(programID, 1024, NULL, infoLog);
            std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                      << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    }
}

} // namespace tod_gl