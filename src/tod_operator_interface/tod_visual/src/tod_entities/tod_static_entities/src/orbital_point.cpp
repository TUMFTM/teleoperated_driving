#include <cmath>
#include "tod_static_entities/orbital_point.hpp"

namespace TodStaticEntities {

tod_gl::Entity OrbitalPoint::create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name, float radius, const std::string& packagePath, const tod_gl::Entity &parentEntity, const tod_gl::CameraComponent &camera) {
    tod_gl::Entity circleEntity = scene->create_entity(name);
    unsigned int shaderProgram = tod_gl::ShaderSystem::create_shader_program(
        (packagePath + "/resources/shaders/shader.vert").c_str(),
        (packagePath + "/resources/shaders/shader.frag").c_str());

    tod_gl::Mesh mesh = generateCircleMesh(radius);
    auto& renderable = circleEntity.add_component<tod_gl::RenderableElementComponent>(shaderProgram, mesh, GL_TRIANGLE_FAN);
    renderable.line_width = .50f;

    auto& transform = circleEntity.get_component<tod_gl::TransformComponent>();
    transform.set_translation(glm::vec3(camera.orbit_point.position.x,camera.orbit_point.position.y, 0.05f));
    transform.setScale(glm::vec3(0.5f));
    transform.set_parent(parentEntity);

    return circleEntity;
}

tod_gl::Mesh OrbitalPoint::generateCircleMesh(float radius) {
    tod_gl::Mesh mesh = tod_gl::Mesh::non_empty_mesh();
    
    const int numVertices = 60;
    const glm::vec3 color = glm::vec3(1.0f, 1.0f, 0.0f);


    mesh.vertices.emplace_back(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f), color);
    
    for (int i = 0; i <= numVertices; ++i) {
        float angle = 2.0 * M_PI * float(i) / float(numVertices);
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        mesh.vertices.emplace_back(glm::vec3(x, y, 0.0f), glm::vec2(0.0f, 0.0f), color);
    }
        
    return mesh;
}

}; // namespace TodStaticEntities