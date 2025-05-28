/**
 * @file object_list_renderer.cpp
 * @brief ObjectListRenderer renders a list of the current boundingboxes percieved by the AV
 * @copyright 2024 TUMFTM
 **/

#include "tod_dynamic_entities/object_list_renderer.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/ros_interface/subscribing_components/predicted_object_component.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/utils/utils.hpp"

namespace TodDynamicEntities {
void ObjectListRenderer::on_create() {
    unsigned int shaderProgram = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shaderTransparent.frag").c_str());
    int expectedNumberOfObjects{50};
    std::vector<unsigned int> indices{0, 1, 2, 0, 2, 3, 0, 3, 4, 3, 4, 7, 4, 5, 6, 4, 6, 7,
                                      1, 2, 5, 2, 5, 6, 0, 1, 5, 0, 5, 4, 3, 2, 6, 3, 6, 7};
    std::vector<unsigned int> initialIndexVec;
    int numberOfVerticesPerObject{8};
    std::vector<tod_gl::Vertex> vertices;
    for (int expectedObjectsIterator = 0; expectedObjectsIterator != expectedNumberOfObjects;
         ++expectedObjectsIterator) {
        for (int iterator = 0; iterator != numberOfVerticesPerObject; ++iterator) {
            vertices.push_back(tod_gl::Vertex());
        }
        initialIndexVec.insert(std::end(initialIndexVec), std::begin(indices), std::end(indices));
    }

    tod_gl::Mesh Mesh(vertices, indices);
    this->add_component<tod_gl::RenderableElementComponent>(shaderProgram, Mesh);
    this->get_component<tod_gl::RenderableElementComponent>().opaque = false;
    this->add_component<tod_gl::ExpirableComponent>(1000);
    this->add_component<tod_gl::DynamicDataComponent>();
    this->get_component<tod_gl::TransformComponent>().is_map_frame = true;

}

void ObjectListRenderer::on_destroy() {}

void ObjectListRenderer::on_update(float) {
    auto& _stateManager = tod_gl::StateManager::get_instance();

     if (!_stateManager.get_toggle_setting("enable_object_list")){
        return;
    }

    auto& dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto& renderable = this->get_component<tod_gl::RenderableElementComponent>();

    if (_stateManager.contains_entity("ObjectRenderer")) {
        if (!_stateManager.should_render_entity("ObjectRenderer")) {
            std::lock_guard<std::mutex> lock(*dynamic.mutex);
            dynamic.has_new_data = true;
            std::for_each(renderable.meshes.begin(), renderable.meshes.end(), [](tod_gl::Mesh& Mesh) {
                Mesh.vertices.clear();
                Mesh.indices.clear();
            });
            return;
        }
    }

    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    this->get_component<tod_gl::ExpirableComponent>().restamp();

    // reset vertex data;
    std::for_each(renderable.meshes.begin(), renderable.meshes.end(), [](tod_gl::Mesh& Mesh) {
        Mesh.vertices.clear();
        Mesh.indices.clear();
    });

    auto entity = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!entity.has_component<tod_gl::PredictedObjectComponent>()) {
        return;
    }

    auto& objectdata = entity.get_component<tod_gl::PredictedObjectComponent>();
    //TODO: Do we run in troubles here, since we return by reference from the PredictedObjectComponent ?
    for (unsigned int index = 0; index != objectdata.get_objects().size(); ++index) {
        const auto& currentObject = objectdata.get_objects().at(index);

        int numberOfVerticesPerObject{8};

        for (int iterator = 0; iterator != numberOfVerticesPerObject; ++iterator) {
            renderable.meshes.front().vertices.push_back(tod_gl::Vertex());
        }

        const auto& pose = currentObject.pose;
        // We only assume BBox
        //" x: the length of the object (BOUNDING_BOX) or diameter (CYLINDER)"
        //" y: the width of the object (BOUNDING_BOX)"
        //" z: the overall height of the object")
        const auto& dimensions = currentObject.dimensions;

        float halfLength = dimensions.x / 2.0f;
        float halfWidth = dimensions.y / 2.0f;
        float height = -dimensions.z;

        // rule-out the detection on the ego-vehicle
        if (std::abs(pose.position.x) < .01f || std::abs(pose.position.y) < .01f)
            continue;

        auto pos = glm::vec3(pose.position.x, pose.position.y, 0);
        glm::vec3 gamePos = tod_gl::TransformSystem::get_instance()->to_game_coordinates(pos);

        glm::mat4 translation = glm::translate(glm::mat4(1.0f), gamePos);

        // const float yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
        // const float rotatedYaw = yaw + glm::radians(glm::pi<float>()/2);

        const auto& q = pose.orientation;
        glm::quat convertedQ = glm::quat(-q.x, -q.y, -q.z, q.w);
        const float yaw = -atan2(2.0 * (convertedQ.y * convertedQ.z + convertedQ.w * convertedQ.x),
                                 convertedQ.w * convertedQ.w - convertedQ.x * convertedQ.x -
                                     convertedQ.y * convertedQ.y + convertedQ.z * convertedQ.z);

        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0.0f, 0.0f, 1.0f));

        fill_vertices_from_vectors_to_the_object_edges(renderable.meshes.front().vertices, index, numberOfVerticesPerObject,
                                                // these values should be based on the object's dimensions
                                                // but the date doesn't have the shape info right now.
                                                // So we use a default shape
                                                // glm::vec3(-2, -1, 0.0f),
                                                // glm::vec3(-2, 1, 0.0f),
                                                // glm::vec3(2, -1, 0.0f),
                                                glm::vec3(-halfLength, -halfWidth, 0.0f),  // rear right
                                                glm::vec3(halfLength, -halfWidth, 0.0f),   // front right
                                                glm::vec3(-halfLength, halfWidth, 0.0f),   // rear left

                                                2);
        transform_vertices(renderable.meshes.front().vertices, index, numberOfVerticesPerObject, translation, rotation);
        add_iIndices(renderable.meshes.front().indices, index, numberOfVerticesPerObject);

        // bounding box color
        glm::vec3 color = get_bounding_box_color_by_class(currentObject);

        for (int iterator = index * numberOfVerticesPerObject; iterator != renderable.meshes.front().vertices.size();
             ++iterator) {
            renderable.meshes.front().vertices.at(iterator).tex_color = color;
        }
    }
}

glm::vec3 ObjectListRenderer::get_bounding_box_color_by_class(const tod_automation_msgs::msg::PredictedObject& currentObject) {
    glm::vec3 color = glm::vec3{1.0f, 1.0f, 1.0f};

    auto classification = currentObject.classification;

    switch (classification) {
        case tod_automation_msgs::msg::PredictedObject::PEDESTRIAN:
            color = glm::vec3{0.9f, 0.25f, 0.25f};  // RED
            break;
        case tod_automation_msgs::msg::PredictedObject::BICYCLE:
            color = glm::vec3{0.92f, 0.75f, 0.25f};  // YELLOW
            break;
        case tod_automation_msgs::msg::PredictedObject::MOTORCYCLE:
            color = glm::vec3{0.92f, 0.75f, 0.25f};  // YELLOW
            break;
        case tod_automation_msgs::msg::PredictedObject::CAR:
            color = glm::vec3{0.25f, 0.8f, 0.9f};  // CYAN
            break;
        case tod_automation_msgs::msg::PredictedObject::BUS:
            color = glm::vec3{0.05f, 0.2f, 0.85f};  // DARK BLUE
            break;
        case tod_automation_msgs::msg::PredictedObject::TRAILER:
            color = glm::vec3{0.05f, 0.2f, 0.85f};  // DARK BLUE
            break;
        case tod_automation_msgs::msg::PredictedObject::TRUCK:
            color = glm::vec3{0.05f, 0.2f, 0.85f};  // DARK BLUE
            break;
        default:
            color = glm::vec3{0.4f, 0.4f, 0.4f};  // GREY
            break;
    }
    return color;
}

//        6  ---  7
//      /       /
//    2  ---  3
//
//        5  ---  4
//     /       /
//    1  ---  0
void ObjectListRenderer::fill_vertices_from_vectors_to_the_object_edges(std::vector<tod_gl::Vertex>& vertices, const int index,
                                                                 const int numberOfVerticesPerObject,
                                                                 const glm::vec3& rearRight, const glm::vec3& rearLeft,
                                                                 const glm::vec3& frontRight, const float height) {
    glm::vec3 frontLeft{rearLeft + frontRight - rearRight};
    glm::vec3 heightVector = glm::vec3(0.0f, 0.0f, height);
    glm::vec3 rearRightHeight(rearRight + heightVector);
    glm::vec3 rearLeftHeight(rearLeft + heightVector);
    glm::vec3 frontLeftHeight(frontLeft + heightVector);
    glm::vec3 frontRightHeight(frontRight + heightVector);
    vertices.at(index * numberOfVerticesPerObject + 0) = tod_gl::Vertex(rearRight);
    vertices.at(index * numberOfVerticesPerObject + 1) = tod_gl::Vertex(rearLeft);
    vertices.at(index * numberOfVerticesPerObject + 2) = tod_gl::Vertex(rearLeftHeight);
    vertices.at(index * numberOfVerticesPerObject + 3) = tod_gl::Vertex(rearRightHeight);
    vertices.at(index * numberOfVerticesPerObject + 4) = tod_gl::Vertex(frontRight);
    vertices.at(index * numberOfVerticesPerObject + 5) = tod_gl::Vertex(frontLeft);
    vertices.at(index * numberOfVerticesPerObject + 6) = tod_gl::Vertex(frontLeftHeight);
    vertices.at(index * numberOfVerticesPerObject + 7) = tod_gl::Vertex(frontRightHeight);
}

void ObjectListRenderer::transform_vertices(std::vector<tod_gl::Vertex>& vertices, const int index,
                                           const int numberOfVerticesPerObject, const glm::mat4& translation,
                                           const glm::mat4& rotation) {
    std::for_each(vertices.begin() + index * numberOfVerticesPerObject, vertices.end(),
                  [&rotation, &translation](tod_gl::Vertex& vertex) {
                      glm::vec4 tmpVector = translation * rotation * glm::vec4(vertex.position, 1.0f);
                      vertex.position = glm::vec3(tmpVector.x, tmpVector.y, tmpVector.z);
                  });
}

void ObjectListRenderer::add_iIndices(std::vector<unsigned int>& indices, const unsigned int index,
                                    const int numberOfVerticesPerObject) {
    unsigned int incre{index * numberOfVerticesPerObject};
    std::vector<unsigned int> indicesOfAppendedObject{
        incre + 0, incre + 1, incre + 2, incre + 0, incre + 2, incre + 3, incre + 0, incre + 3, incre + 4,
        incre + 3, incre + 4, incre + 7, incre + 4, incre + 5, incre + 6, incre + 4, incre + 6, incre + 7,
        incre + 1, incre + 2, incre + 5, incre + 2, incre + 5, incre + 6, incre + 0, incre + 1, incre + 5,
        incre + 0, incre + 5, incre + 4, incre + 3, incre + 2, incre + 6, incre + 3, incre + 6, incre + 7};
    indices.insert(std::end(indices), std::begin(indicesOfAppendedObject), std::end(indicesOfAppendedObject));
}

}  // namespace TodDynamicEntities