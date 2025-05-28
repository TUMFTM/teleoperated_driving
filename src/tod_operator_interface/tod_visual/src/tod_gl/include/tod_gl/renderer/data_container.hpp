/**
 * @file data_container.hpp
 * @brief Container file for all render related data structs e.g. vertices, textures, buffers etc. meshes
 * @copyright 2024 TUMFTM

 * @brief Data Structure Overview
 * 
 * The rendering system uses several key data structures for efficient GPU rendering:
 * 
 * 1. Vertex - Fundamental building block for 3D geometry
 *    - Position (vec3): 3D coordinates in model space
 *    - Texture Coordinates (vec2): UV mapping for textures
 *    - Texture Color (vec3): Per-vertex color information
 * 
 * 2. Mesh - Complete 3D object representation
 *    - Vertices: Vector of Vertex structures
 *    - Indices: Optional vector for indexed rendering
 *    - Textures: Vector of associated textures
 *    - OpenGL buffer objects (VAO, VBO, IBO)
 * 
 * 3. Texture - OpenGL texture representation
 *    - Texture ID, dimensions, and format
 *    - Name for shader uniform binding
 *    - Internal format and pixel format specifications
 * 
 * 4. Buffer - Abstract OpenGL buffer
 *    - Buffer ID and target type (array buffer, element buffer, etc.)
 *    - Usage hint (static, dynamic, stream)
 * 
 * The rendering process involves:
 * 1. Creating and configuring these structures
 * 2. Uploading data to GPU memory via OpenGL buffers
 * 3. Binding appropriate buffers and textures before drawing
 * 4. Issuing draw commands with proper render modes
 */

#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace tod_gl {

/**
 * @brief Represents a single vertex with position, texture coordinates, and color.
 * 
 * This struct defines the fundamental building block for 3D geometry in OpenGL.
 * It stores all necessary data for rendering a vertex including its position in 3D space,
 * mapping coordinates for textures, and color information.
 */
struct Vertex {
    glm::vec3 position;
    glm::vec2 tex_coord;
    glm::vec3 tex_color;

    Vertex(const glm::vec3 &position = glm::vec3(0.0f, 0.0f, 0.0f), const glm::vec2 &texCoord = glm::vec2(0.0f, 0.0f),
           const glm::vec3 &texColor = glm::vec3(0.0f, 0.0f, 0.0f))
        : position(position), tex_coord(texCoord), tex_color(texColor) {}
};

/**
 * @brief Represents an OpenGL texture object with its properties.
 * 
 * This struct encapsulates all the data needed to handle textures in OpenGL.
 * It stores the texture ID, dimensions, format information, and type which are
 * essential for properly binding and using textures with GLAD/OpenGL.
 */
struct Texture {
    unsigned int id{0}, width{0}, height{0};
    std::string name{""};
    GLenum type{GL_NONE};
    GLenum internal_format{GL_RED};
    GLenum format{GL_RED};

    Texture(const unsigned int width = 0, const unsigned int height = 0, const std::string &name = "",
            const GLenum type = GL_NONE, GLenum internalFormat = GL_RED, const GLenum format = GL_RED)
        : width{width}, height{height}, name{name}, type{type}, format{format}, internal_format(internalFormat) {}
};

/**
 * @brief Represents an OpenGL buffer object.
 * 
 * This struct abstracts the OpenGL buffer concept, used for storing vertex data,
 * indices, or other rendering information. It maintains the buffer ID and information
 * about its usage pattern and target type, which are critical for proper
 * memory management in the OpenGL pipeline.
 */
struct Buffer {
    GLuint id{0};
    GLenum target, usage;
    Buffer() = default;
    Buffer(const GLenum target, const GLenum usage) : target{target}, usage{usage} {}
};

/**
 * @brief Represents an OpenGL Vertex Array Object (VAO).
 * 
 * This struct provides a container for the OpenGL VAO ID. VAOs store all of the state
 * needed to supply vertex data to the rendering pipeline, making it efficient
 * to switch between different vertex data and formats.
 */
struct VertexArray {
    GLuint id{0};
};

/**
 * @brief Represents a complete 3D mesh with vertices, indices, textures, and buffer objects.
 * 
 * This struct combines all the elements needed to render a complete 3D object in OpenGL.
 * It stores the vertex data, index data for optimized rendering, associated textures,
 * and the necessary OpenGL buffer objects (VAO, VBO, IBO) required for efficient
 * rendering with the GLAD/OpenGL pipeline.
 */
struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;
    VertexArray vertex_array_object;
    Buffer vertex_buffer, index_buffer;

    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices = std::vector<unsigned int>(),
         std::vector<Texture> textures = std::vector<Texture>())
        : vertices(vertices),
          indices(indices),
          textures(textures),
          vertex_buffer(GL_ARRAY_BUFFER, GL_STATIC_DRAW),
          index_buffer(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW) {
        if (vertices.empty()) {
            vertices = vector_of_three_zero_vertices();
        }
    }

    static Mesh non_empty_mesh() {
        Mesh mesh(Mesh::vector_of_three_zero_vertices());
        return mesh;
    }

    static std::vector<Vertex> vector_of_three_zero_vertices() {
        std::vector<Vertex> vertices;
        for (int i = 0; i < 3; ++i)
            vertices.emplace_back(Vertex(glm::vec3(0.0f, 0.0f, 0.0f)));
        return vertices;
    }
};

/**
 * @brief Represents a single text character for font rendering.
 * 
 * This struct stores all the information needed to render a single character
 * in OpenGL. It includes the texture containing the glyph, size information,
 * bearing offsets, and advance width which are essential for proper text
 * positioning and rendering.
 */
struct Character {
    Texture tex;
    glm::ivec2 size;
    glm::ivec2 bearing;
    unsigned int advance;
};

/**
 * @brief Represents a point in a trajectory with pose and twist information.
 * 
 * This struct defines a complete state for an object in motion, combining
 * both position/orientation (pose) and linear/angular velocities (twist).
 * It's designed to support trajectory visualization and motion planning
 * in the OpenGL rendering environment.
 */
struct TrajectoryPoint {
    struct Pose {
        glm::vec3 position;
        glm::quat orientation;

        Pose() : position(0.f), orientation(0.f, 0.f, 0.f, 1.f) {}
        Pose(const glm::vec3& pos, const glm::quat& ori) : position(pos), orientation(ori) {} 
    };

    struct Twist {
        glm::vec3 linear;
        glm::vec3 angular;

        Twist() : linear(0.f), angular(0.f) {}
        Twist(const glm::vec3& lin, const glm::vec3& ang) : linear(lin), angular(ang) {}
    };
    
    Pose pose;
    Twist twist;

    TrajectoryPoint() : pose(), twist() {};
    TrajectoryPoint(const glm::vec3& pos, const glm::quat& ori, glm::vec3 lin, glm::vec3 ang) 
        : pose(pos, ori), twist(lin, ang) {};
};

}  // namespace tod_gl