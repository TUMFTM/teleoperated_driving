/**
 * @file path_renderer.hpp
 * @brief Renders the vehicle's driving path and progress tics.
 *
 * Declares the PathRenderer class which displays the path the vehicle should follow.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/scriptable_entity.hpp"

namespace TodDynamicEntities {

/**
 * @class PathRenderer
 * @brief Templated class for rendering the vehicle's driving path.
 *
 * Displays the path the vehicle should follow, along with progress tics.
 *
 * @tparam PathLike The type representing the path data.
 */
template<typename PathLike>
class PathRenderer : public tod_gl::ScriptableEntity
{
    public:
        /**
         * @brief Constructs a new PathRenderer object.
         *
         * @param stateKey Identifier for the scene manager state.
         * @param colors Color for rendering the path (default: white).
         * @param lineWidth Width of the rendered path line.
         * @param zpos Z-position offset for the path.
         */
        PathRenderer(std::string stateKey, 
                     const glm::vec3& colors = glm::vec3(1.0f),
                     float lineWidth = 2.7f,
                     float zpos = .05f)
                : _scene_manager_key(std::move(stateKey))
                , _color(colors)
                , _lineWidth(lineWidth)
                , _zpos(zpos) {}

        /**
         * @brief Initializes resources for path rendering.
         */
        virtual void on_create() override;

        /**
         * @brief Releases resources used for path rendering.
         */
        virtual void on_destroy() override;

        /**
         * @brief Updates the rendered path.
         *
         * @param delta_time Time elapsed since the last update.
         */
        virtual void on_update(float delta_time) override;

    private:
        /// Width of the rendered path line.
        float _lineWidth;

        /// Z-axis offset for rendering the path.
        float _zpos;

        /// Color of the rendered path.
        glm::vec3 _color;

        /// Identifier for the scene manager state.
        std::string _scene_manager_key;
};

}  // namespace TodDynamicEntities
