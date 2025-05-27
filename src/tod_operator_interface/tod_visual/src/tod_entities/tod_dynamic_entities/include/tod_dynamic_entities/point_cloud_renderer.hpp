/**
 * @file point_cloud_renderer.hpp
 * @brief Renders a pointcloud around the vehicle's base footprint.
 *
 * Declares the PointCloudRenderer class for rendering a single-colored pointcloud.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/scriptable_entity.hpp"

namespace TodDynamicEntities {

/**
 * @class PointCloudRenderer
 * @brief Renders the pointcloud.
 */
class PointCloudRenderer : public tod_gl::ScriptableEntity {
  public:
    /**
     * @brief Default constructor.
     */
    PointCloudRenderer() = default;

    /**
     * @brief Initializes resources for pointcloud rendering.
     */
    virtual void on_create() override;

    /**
     * @brief Releases resources used for pointcloud rendering.
     */
    virtual void on_destroy() override;

    /**
     * @brief Updates the pointcloud rendering.
     *
     * @param delta_time Time elapsed since the last update.
     */
    virtual void on_update(float delta_time) override;

  private:
    /**
     * @brief Renders the LiDAR pointcloud.
     *
     * @param mesh The mesh to update with pointcloud data.
     */
    void render_lidar_point_cloud(tod_gl::Mesh &mesh);
};

}  // namespace TodDynamicEntities
