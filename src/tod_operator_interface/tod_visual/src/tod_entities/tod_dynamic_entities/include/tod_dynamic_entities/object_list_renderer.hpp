/**
 * @file object_list_renderer.hpp
 * @brief ObjectListRenderer renders a list of the current bounding boxes perceived by the AV.
 *
 * This file contains the declaration of the ObjectListRenderer class, which is responsible for visualizing
 * detected objects—such as vehicles and pedestrians—as colored bounding boxes in a 3D scene.
 * The renderer creates, updates, and destroys the object representations based on the bounding boxes.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/scriptable_entity.hpp"

#include "tod_automation_msgs/msg/predicted_object.hpp"

namespace TodDynamicEntities {

/**
 * @class ObjectListRenderer
 * @brief Renders a list of the current bounding boxes perceived by the AV.
 *
 * The ObjectListRenderer class visualizes entities detected by the autonomous vehicle (AV), such as vehicles
 * and pedestrians, as colored bounding boxes. The color of each bounding box is determined by the object's class.
 * The class provides methods to initialize, update, and destroy the rendering resources.
 */
class ObjectListRenderer : public tod_gl::ScriptableEntity {
  public:
    /**
     * @brief Default constructor.
     */
    ObjectListRenderer() = default;

    /**
     * @brief Called when the renderer is created.
     *
     * Initializes the necessary resources for rendering the bounding boxes.
     */
    virtual void on_create() override;

    /**
     * @brief Called when the renderer is destroyed.
     *
     * Releases resources allocated for the bounding box rendering.
     */
    virtual void on_destroy() override;

    /**
     * @brief Updates the renderer each frame.
     *
     * Recalculates the positions and colors of the bounding boxes based on the current list of detected objects.
     *
     * @param delta_time Time elapsed since the last update.
     */
    virtual void on_update(float delta_time) override;

  private:
    /**
     * @brief Determines the color of the bounding box based on the object class.
     *
     * Returns a color corresponding to the class of the predicted object (e.g., vehicle, pedestrian).
     *
     * @param currentObject The predicted object for which the bounding box color is determined.
     * @return glm::vec3 The color to be used for the bounding box.
     */
    glm::vec3 get_bounding_box_color_by_class(const tod_automation_msgs::msg::PredictedObject& currentObject);

    /**
     * @brief Fills vertices for the edges of an object's bounding box.
     *
     * Populates the vertex array with coordinates representing the edges of the bounding box.
     *
     * @param vertices The vector to be filled with vertex data.
     * @param index The starting index for the current object.
     * @param numberOfVerticesPerObject The number of vertices for each object.
     * @param rearRight The rear right corner of the bounding box.
     * @param rearLeft The rear left corner of the bounding box.
     * @param frontRight The front right corner of the bounding box.
     * @param height The height of the bounding box.
     */
    static void fill_vertices_from_vectors_to_the_object_edges(std::vector<tod_gl::Vertex>& vertices, const int index,
                                                        const int numberOfVerticesPerObject, const glm::vec3& rearRight,
                                                        const glm::vec3& rearLeft, const glm::vec3& frontRight,
                                                        const float height);

    /**
     * @brief Transforms the vertices of a bounding box.
     *
     * Applies translation and rotation transformations to the vertices of the bounding box.
     *
     * @param vertices The vector containing vertex data.
     * @param index The starting index for the current object.
     * @param numberOfVerticesPerObject The number of vertices for each object.
     * @param translation The translation matrix.
     * @param rotation The rotation matrix.
     */
    static void transform_vertices(std::vector<tod_gl::Vertex>& vertices, const int index,
                                  const int numberOfVerticesPerObject, const glm::mat4& translation,
                                  const glm::mat4& rotation);

    /**
     * @brief Adds indices for rendering the bounding box.
     *
     * Populates the index array with indices corresponding to the vertices of the bounding box.
     *
     * @param indices The vector to be filled with index data.
     * @param index The starting index for the current object.
     * @param numberOfVerticesPerObject The number of vertices for each object.
     */
    static void add_iIndices(std::vector<unsigned int>& indices, const unsigned int index,
                           const int numberOfVerticesPerObject);
};

}  // namespace TodDynamicEntities
