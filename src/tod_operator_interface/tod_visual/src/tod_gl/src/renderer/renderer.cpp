/**
 * @file renderer.cpp
 * @brief TODO: BRief
 * @copyright 2024 TUMFTM 
 **/

#include "tod_gl/renderer/renderer.hpp"

#include <iostream>
#include <mutex>
#include <string>
#include <utility>
#include <unistd.h>

#include "tod_gl/events/mouse_events.hpp"
#include "tod_gl/renderer/renderer_command.hpp"
#include "tod_gl/systems/shader_system.hpp"

#include "entt/entt.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

// std::unique_ptr<Renderer::SceneData> Renderer::s_SceneData = std::make_unique<Renderer::SceneData>();
namespace tod_gl {

void Renderer::generate_frame_buffer(FrameBufferComponent &framebuffer, RenderableElementComponent &renderable) {
    tod_gl::RenderCommand::ForFramebuffer::generate_and_bind(&framebuffer.frame_buffer_object);
    RenderCommand::ForRenderbuffer::create(&framebuffer.render_buffer_object, GL_DEPTH24_STENCIL8, framebuffer.render_width,
                                           framebuffer.render_height);
    RenderCommand::ForFramebuffer::attach_texture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                                 renderable.meshes.front().textures.front().id);
    RenderCommand::ForFramebuffer::attach_renderbuffer(framebuffer.render_buffer_object, GL_DEPTH_STENCIL_ATTACHMENT);
    RenderCommand::ForFramebuffer::check_and_unbind();
}

void Renderer::generate_meshes(RenderableElementComponent &renderable) {
    for (Mesh &mesh : renderable.meshes) {
        if (mesh.vertices.empty()) {
            // TODO(Andi): reintroduce print
            // ROS_ERROR("%s: Trying to generate mesh with empty vertices - filling up",
            //           ros::this_node::get_name().c_str());
            mesh.vertices = Mesh::vector_of_three_zero_vertices();
        }
        RenderCommand::ForVertexArray::generate_and_bind(mesh.vertex_array_object);
        RenderCommand::ForBuffer::generate_bind_and_upload(mesh.vertex_buffer, mesh.vertices.data(),
                                                        (uint32_t)mesh.vertices.size() * sizeof(Vertex));
        if (!mesh.indices.empty()) {
            RenderCommand::ForBuffer::generate_bind_and_upload(
                mesh.index_buffer, mesh.indices.data(), (uint32_t)mesh.indices.size() * sizeof(mesh.indices.front()));
        }
        RenderCommand::ForVertexAttributePtr::set_and_enable(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
        RenderCommand::ForVertexAttributePtr::set_and_enable(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                                           (void *)(sizeof(Vertex::position)));
        RenderCommand::ForVertexAttributePtr::set_and_enable(
            2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)(sizeof(Vertex::position) + sizeof(Vertex::tex_coord)));

        // unbind - order matters (VAO first)
        RenderCommand::ForVertexArray::unbind(mesh.vertex_array_object);
        RenderCommand::ForBuffer::unbind(mesh.index_buffer);
        RenderCommand::ForBuffer::unbind(mesh.vertex_buffer);
    }
}

void Renderer::generate_texture(Texture &texture, const void *data, const int ShaderProgram, const int uniformId) {
    RenderCommand::ForTexture::generate_active_and_bind(texture, 0);
    RenderCommand::ForTexture::bind_image(texture, data);
    RenderCommand::ForTexture::set_parameter(texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    RenderCommand::ForTexture::set_parameter(texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    RenderCommand::ForTexture::set_parameter(texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    RenderCommand::ForTexture::set_parameter(texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    RenderCommand::ForTexture::unbind(texture);

    ShaderSystem::set_shader_program_int(ShaderProgram, texture.name, uniformId);
}

void Renderer::delete_texture(Texture &texture) {
    RenderCommand::ForTexture::delete_texture(texture);
}

void Renderer::create_buffer(Buffer &buffer, void *data, uint32_t size) {
    RenderCommand::ForBuffer::generate_bind_and_upload(buffer, data, size);
    RenderCommand::ForBuffer::unbind(buffer);
}

void Renderer::render_meshes(const RenderableElementComponent &renderable) {
    if (!renderable.dynamic_show || !renderable.static_show)
        return;
    ShaderSystem::use_shader_program(renderable.shader_program);
    RenderCommand::set_line_width(renderable.line_width);
    RenderCommand::set_point_size(renderable.point_size);
    for (const Mesh &mesh : renderable.meshes) {
        if (!mesh.vertex_array_object.id) {
            // TODO(Andi): reintroduce print
            // ROS_ERROR("%s: in renderMesh() no VAO to bind - skipping mesh",
            //           ros::this_node::get_name().c_str());
            continue;
        }
        if (mesh.vertices.empty())
            continue;
        for (int i = 0; i < mesh.textures.size(); ++i) {
            if (mesh.textures.at(i).type == 0)
                continue;
            RenderCommand::ForTexture::active_and_bind(mesh.textures.at(i), i);
        }
        RenderCommand::ForVertexArray::bind(mesh.vertex_array_object);
        if (!mesh.indices.empty()) {
            RenderCommand::Draw::Elements(renderable.render_mode, (uint32_t)mesh.indices.size(), GL_UNSIGNED_INT, 0);
        } else {
            RenderCommand::Draw::Arrays(renderable.render_mode, 0, (uint32_t)mesh.vertices.size());
        }
        RenderCommand::ForVertexArray::unbind(mesh.vertex_array_object);
    }
    ShaderSystem::use_shader_program(0);
}

void Renderer::update_texture(const Texture &tex, const unsigned int texIdx, const Buffer &buf,
                             const unsigned int xOffset, const unsigned int yOffset, const unsigned width,
                             const unsigned height, void *data) {
    RenderCommand::enable(GL_TEXTURE_RECTANGLE);
    RenderCommand::ForBuffer::bind_and_upload(buf, data, width * height * 3);
    RenderCommand::ForTexture::active_and_bind(tex, texIdx);
    RenderCommand::ForTexture::bind_sub_image(tex, xOffset, yOffset, width, height, 0);
    //RenderCommand::ForTexture::unbind(tex);
    RenderCommand::ForBuffer::unbind(buf);

}

void Renderer::generate_textures(RenderableElementComponent &renderable) {
    for (Mesh &mesh : renderable.meshes) {
        for (int i = 0; i < mesh.textures.size(); ++i) {
            if (renderable.meshes.front().textures.at(i).id == 0) {
                generate_texture(renderable.meshes.front().textures.at(i), 0, renderable.shader_program, i);
            }
        }
    }
}

void Renderer::update_meshes(const RenderableElementComponent &renderable, DynamicDataComponent &dynamic) {
    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    if (!dynamic.has_new_data)
        return;

    for (const Mesh &mesh : renderable.meshes) {
        if (!mesh.vertex_array_object.id) {
            // TODO(Andi): reintroduce print
            // ROS_ERROR("%s: in uploadBufferDataToGPU() no VAO to bind - skipping mesh",
            //           ros::this_node::get_name().c_str());
            continue;
        }
        if (mesh.vertices.empty())
            continue;
        for (uint32_t i = 0; i < mesh.textures.size(); ++i)
            RenderCommand::ForTexture::active_and_bind(mesh.textures.at(i), i);
        RenderCommand::ForVertexArray::bind(mesh.vertex_array_object);
        RenderCommand::ForBuffer::bind_and_upload(mesh.vertex_buffer, mesh.vertices.data(),
                                                (uint32_t)mesh.vertices.size() * sizeof(mesh.vertices.front()));
        if (!mesh.indices.empty()) {
            if (!mesh.index_buffer.id) {
                // TODO(Andi): reintroduce print
                // ROS_WARN("%s: in uploadBufferToGPU() - no EBO to bind but indices stored",
                //          ros::this_node::get_name().c_str());
            }
            RenderCommand::ForBuffer::bind_and_upload(mesh.index_buffer, mesh.indices.data(),
                                                    (uint32_t)mesh.indices.size() * sizeof(mesh.indices.front()));
        }

        // unbind - order matters (VAO first)
        RenderCommand::ForVertexArray::unbind(mesh.vertex_array_object);
        RenderCommand::ForBuffer::unbind(mesh.index_buffer);
        RenderCommand::ForBuffer::unbind(mesh.vertex_buffer);
    }
    dynamic.has_new_data = false;
}

void Renderer::setup_rendering_for_framebuffer(const FrameBufferComponent &framebuffer) {
    RenderCommand::ForFramebuffer::bind(framebuffer.frame_buffer_object);
    RenderCommand::enable(GL_DEPTH_TEST);
    RenderCommand::enable(GL_MULTISAMPLE);
    RenderCommand::set_viewport(0, 0, framebuffer.render_width, framebuffer.render_height);
    RenderCommand::clear();
}

void Renderer::PrintGLError(const char *file, int line) {
    GLenum errorCode;
    while ((errorCode = glGetError()) != GL_NO_ERROR) {
        std::string error;

        switch (errorCode) {
            case GL_INVALID_ENUM:
                error = "INVALID_ENUM";
                break;
            case GL_INVALID_VALUE:
                error = "INVALID_VALUE";
                break;
            case GL_INVALID_OPERATION:
                error = "INVALID_OPERATION";
                break;
            case GL_STACK_OVERFLOW:
                error = "STACK_OVERFLOW";
                break;
            case GL_STACK_UNDERFLOW:
                error = "STACK_UNDERFLOW";
                break;
            case GL_OUT_OF_MEMORY:
                error = "OUT_OF_MEMORY";
                break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:
                error = "INVALID_FRAMEBUFFER_OPERATION";
                break;
        }
        std::cout << error << " | " << file << " (" << line << ")" << std::endl;
    }
}
#define PrintGLError() PrintGLError(__FILE__, __LINE__)

}  // namespace tod_gl