/**
 * @file lanelet_map_renderer.hpp
 * @brief LaneletMapRenderer renders a lanelet2 map in a 3D visualization.
 *
 * This file contains the declaration of the LaneletMapRenderer class, which is responsible
 * for loading a lanelet2 map, calculating offsets, updating mesh data, and rendering the map
 * in a 3D scene.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/scriptable_entity.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

using namespace lanelet;

namespace TodDynamicEntities {

/**
 * @class LaneletMapRenderer
 * @brief Renders a lanelet2 map in a 3D visualization.
 *
 * The LaneletMapRenderer class is responsible for loading a lanelet2 map from a specified file,
 * calculating the required offsets based on the map origin, updating mesh data, and rendering
 * the map in a 3D visualization environment. It derives from tod_gl::ScriptableEntity to integrate
 * with the scene management framework.
 */
class LaneletMapRenderer : public tod_gl::ScriptableEntity {
  public:
    /**
     * @brief Constructs a new LaneletMapRenderer object.
     *
     * @param map_path The file path to the lanelet2 map.
     * @param map_origin A vector representing the map's origin in the 3D space.
     */
    LaneletMapRenderer(const std::string& map_path, const std::vector<double>& map_origin) : 
      map_path_(map_path), 
      map_origin_(map_origin) { };

    /**
     * @brief Called when the entity is created.
     *
     * Initializes resources, loads the lanelet map, and prepares the rendering data.
     */
    virtual void on_create() override;

    /**
     * @brief Updates the entity.
     *
     * This function is called every frame to update the rendered map based on any changes in
     * the environment or internal state.
     *
     * @param delta_time The elapsed time since the last update.
     */
    virtual void on_update(float delta_time) override;

  private:
    /**
     * @brief Flag indicating whether the lanelet mesh is empty.
     *
     * This flag is used to check if the mesh data for the lanelet map has been initialized.
     */
    bool lanelet_mesh_empty_ = true;

    /**
     * @brief File path to the lanelet map.
     */
    std::string map_path_ = "";

    /**
     * @brief Origin of the map in 3D space.
     *
     * The origin is used for offset calculations to correctly position the map in the scene.
     */
    std::vector<double> map_origin_{0.0, 0.0, 0.0};

    /**
     * @brief Pointer to the loaded lanelet map.
     */
    lanelet::LaneletMapConstPtr map_ptr_;

    /**
     * @brief Line width used for rendering the map lines.
     */
    float line_width_ = 0.15f;

    /**
     * @brief Loads the lanelet map from the specified file.
     *
     * This function reads the map file and initializes the map pointer.
     */
    void load_map();  

    /**
     * @brief Calculates the offset based on the map origin.
     *
     * Computes the necessary offsets to correctly position the map in the 3D scene.
     */
    void calc_offset();                                      

    /**
     * @brief Updates the mesh data for rendering.
     *
     * Processes the lanelet map data to create or update the mesh representation.
     *
     * @param meshes A vector of meshes to be updated with the new lanelet map data.
     */
    void update_meshes(std::vector<tod_gl::Mesh>& meshes);

    /**
     * @brief Adds a line to the mesh.
     *
     * Converts a lanelet line string into a mesh line for 3D visualization.
     *
     * @param linestring The lanelet line string representing a road line.
     * @param meshes The vector of meshes to which the line will be added.
     * @param color The color of the line in the visualization.
     */
    void add_line_to_mesh(const lanelet::ConstLineString3d& linestring, 
                          std::vector<tod_gl::Mesh>& meshes,
                          glm::vec3& color);
};

}  // namespace TodDynamicEntities
