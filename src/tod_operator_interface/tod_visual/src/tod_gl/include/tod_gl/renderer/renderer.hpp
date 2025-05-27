/**
 * @file renderer.hpp
 * @brief Renerer abstraction for the render commands for obejct in the scene 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/components.hpp"

namespace tod_gl {

class Renderer {
  public:
    /**
     * @brief Generates OpenGL buffer objects for mesh geometry
     * 
     * Creates and configures vertex array objects (VAOs), vertex buffer objects (VBOs),
     * and index buffer objects (IBOs) for each mesh in the renderable component.
     * This method sets up the vertex attribute pointers to define the data layout
     * (position, texture coordinates, and color) for the shaders.
     * 
     * @param renderable The renderable component containing meshes to generate buffers for
     */
    static void generate_meshes(RenderableElementComponent &renderable);
    
    /**
     * @brief Creates and configures a framebuffer for off-screen rendering
     * 
     * Sets up a framebuffer object (FBO) with color and depth-stencil attachments.
     * The color attachment uses a texture from the renderable component, allowing
     * the rendered output to be used as a texture in subsequent rendering passes.
     * 
     * @param framebuffer The framebuffer component to be configured
     * @param renderable The renderable component containing the texture for color attachment
     */
    static void generate_frame_buffer(FrameBufferComponent &framebuffer, RenderableElementComponent &renderable);
    /**
     * @brief Creates OpenGL texture objects for all textures in the renderable component
     * 
     * Iterates through all meshes and their textures, generating and configuring
     * OpenGL texture objects for any that haven't been initialized yet.
     * 
     * @param renderable The renderable component containing textures to be generated
     */
    static void generate_textures(RenderableElementComponent &renderable);
    
    /**
     * @brief Creates and configures a single OpenGL texture object
     * 
     * Generates a new texture, sets filtering and wrapping parameters, and binds it
     * to a shader uniform. This method is used for creating individual textures
     * rather than processing all textures in a renderable component.
     * 
     * @param texture The texture object to be configured
     * @param data Pointer to the texture image data (can be NULL for empty texture)
     * @param ShaderProgram ID of the shader program to bind the texture to
     * @param uniformId Texture unit index for the shader binding
     */
    static void generate_texture(Texture &texture, const void *data, const int ShaderProgram, const int uniformId);
    
    /**
     * @brief Deletes an OpenGL texture object
     * 
     * Releases the GPU resources associated with a texture.
     * 
     * @param texture The texture object to be deleted
     */
    static void delete_texture(Texture &texture);
        /**
     * @brief Creates an OpenGL buffer object for generic data
     * 
     * Generates, binds, and uploads data to a buffer object. This is a general-purpose
     * method for creating buffers that aren't specifically vertex or index buffers.
     * 
     * @param buffer The buffer object to be created
     * @param data Pointer to the data to be uploaded
     * @param size Size of the data in bytes
     */
    static void create_buffer(Buffer &buffer, void *data, uint32_t size);

    // on update

    /**
     * @brief Updates mesh data in GPU memory for dynamic meshes
     * 
     * Uploads new vertex and index data to the GPU for meshes that have changed.
     * This method is used for dynamic geometry that changes during runtime
     * and is guarded by a mutex for thread safety.
     * 
     * @param renderable The renderable component containing meshes to be updated
     * @param dynamic The dynamic data component marking whether data has changed
     */
    static void update_meshes(const RenderableElementComponent &renderable, DynamicDataComponent &dynamic);
    
    /**
     * @brief Prepares OpenGL state for rendering to a specific framebuffer
     * 
     * Binds the framebuffer, enables depth testing and multisampling, sets the viewport
     * dimensions, and clears the color and depth buffers. This method is called before
     * rendering to each framebuffer.
     * 
     * @param framebuffer The framebuffer component to render to
     */ 
    static void setup_rendering_for_framebuffer(const FrameBufferComponent &framebuffer);
        /**
     * @brief Renders all meshes in the renderable component
     * 
     * Activates the shader program, binds textures, sets line width and point size,
     * and issues draw calls for each mesh. This method handles both indexed and non-indexed
     * drawing modes and respects visibility flags.
     * 
     * @param renderable The renderable component containing meshes to be rendered
     */
    static void render_meshes(const RenderableElementComponent &renderable);
    
    /**
     * @brief Updates a portion of a texture with new data
     * 
     * Uses a pixel buffer object (PBO) to efficiently transfer new image data to
     * a region of an existing texture. This method is typically used for updating
     * video textures or other dynamic image content.
     * 
     * @param tex The texture to be updated
     * @param texIdx Texture unit index for binding
     * @param buf Buffer containing the new pixel data
     * @param xOffset X-coordinate offset in the texture
     * @param yOffset Y-coordinate offset in the texture
     * @param width Width of the updated region
     * @param height Height of the updated region
     * @param data Pointer to the new pixel data
     */
    static void update_texture(const Texture &tex, const unsigned int texIdx, const Buffer &buf,
                              const unsigned int xOffset, const unsigned int yOffset, const unsigned width,
                              const unsigned height, void *data);
        /**
     * @brief Updates a layer of a 3D or array texture with new data
     * 
     * Similar to update_texture, but specifically for updating a single layer
     * of a 3D or array texture.
     * 
     * @param tex The texture to be updated
     * @param texIdx Texture unit index for binding
     * @param buf Buffer containing the new pixel data
     * @param xOffset X-coordinate offset in the texture
     * @param yOffset Y-coordinate offset in the texture
     * @param width Width of the updated region
     * @param height Height of the updated region
     * @param data Pointer to the new pixel data
     */
    static void update_texture_layer(const Texture &tex, const unsigned int texIdx, const Buffer &buf,
                                   const unsigned int xOffset, const unsigned int yOffset, const unsigned width,
                                   const unsigned height, void *data);
    
    
    /**
     * @brief Updates a texture using a framebuffer for data transfer
     * 
     * Alternative method for updating texture data that uses a framebuffer
     * object for the data transfer instead of a pixel buffer object.
     * 
     * @param tex The texture to be updated
     * @param texIdx Texture unit index for binding
     * @param fbo Framebuffer object ID for the transfer
     * @param xOffset X-coordinate offset in the texture
     * @param yOffset Y-coordinate offset in the texture
     * @param width Width of the updated region
     * @param height Height of the updated region
     * @param data Pointer to the new pixel data
     */
    static void update_texture(const Texture &tex, const unsigned int texIdx, const uint32_t fbo,
                              const unsigned int xOffset, const unsigned int yOffset, const unsigned width,
                              const unsigned height, void *data);

    // debug
    static void PrintGLError(const char *file, int line);
};

}  // namespace tod_gl