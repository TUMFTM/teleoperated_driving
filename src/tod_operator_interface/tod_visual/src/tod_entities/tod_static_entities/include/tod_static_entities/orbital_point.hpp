/**
 * @file orbital_point.hpp
 * @brief Creates an orbital point entity using a circle mesh.
 *
 * This entity visualizes an orbital point, generated as a circle mesh,
 * and attaches it to a parent entity.
 * 
 * @copyright 2024 TUMFTM
 **/

#pragma once
#include <cmath>
#include <glm/glm.hpp>
#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/scene.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"

namespace TodStaticEntities {

/**
 * @class OrbitalPoint
 * @brief Static entity for creating an orbital point.
 *
 * Generates a circular mesh to represent an orbital point and creates an entity
 * with the given parameters.
 */
class OrbitalPoint {
    public:
        /**
         * @brief Creates an OrbitalPoint entity.
         *
         * @param scene Shared pointer to the scene.
         * @param name Name of the entity.
         * @param radius Radius of the orbital point.
         * @param packagePath Path to package resources.
         * @param parentEntity Parent entity to attach the orbital point.
         * @param camera Camera component for reference.
         * @return tod_gl::Entity The created OrbitalPoint entity.
         */
        static tod_gl::Entity create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name, float radius, const std::string& packagePath, const tod_gl::Entity &parentEntity, const tod_gl::CameraComponent &camera);

    private:
        OrbitalPoint() = default;

        /**
         * @brief Generates a circle mesh.
         *
         * @param radius Radius of the circle.
         * @return tod_gl::Mesh A mesh representing the circle.
         */
        static tod_gl::Mesh generateCircleMesh(float radius);
};

} // namespace TodStaticEntities
