/**
 * @file display.hpp
 * @brief 3D Car HUD with text.
 *
 * Provides a HUD to display vehicle information using FreeType.
 * 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ft2build.h>
#include FT_FREETYPE_H

#include "tod_gl/renderer/renderer.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/scene.hpp"
#include "tod_gl/systems/shader_system.hpp"

#include "tod_vehicle_msgs/VehicleEnums.h"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/secondary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/primary_vehicle_state.hpp"
#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"

namespace TodStaticEntities {

/**
 * @class Display
 * @brief 3D Car HUD for vehicle info.
 */
class Display {
  public:
    /// Create the Display entity.
    static tod_gl::Entity create(std::shared_ptr<tod_gl::Scene> scene, std::string name, tod_gl::Entity parent,
                                 const std::string& packagePath);

    /// Update HUD with current speed.
    static void onSpeedUpdate(const tod_vehicle_msgs::msg::PrimaryVehicleState::ConstSharedPtr& msg, tod_gl::Entity& entity);

    /// Update HUD with desired speed.
    static void onDesiredSpeedUpdate(const tod_vehicle_msgs::msg::PrimaryControlCmd::ConstSharedPtr& msg,
                                     tod_gl::Entity& entity);

    /// Update HUD with current gear.
    static void onGearUpdate(const tod_vehicle_msgs::msg::SecondaryVehicleState::ConstSharedPtr& msg, tod_gl::Entity& entity);

    /// Update HUD with desired gear.
    static void onDesiredGearUpdate(const tod_vehicle_msgs::msg::SecondaryControlCmd::ConstSharedPtr& msg,
                                    tod_gl::Entity& entity);

  private:
    Display() = default;

    static void GenerateCharactersWithTextureForDisplay(const std::string& pathToTTF,
                                                        tod_gl::CharacterMapComponent& charMap,
                                                        tod_gl::RenderableElementComponent& renderable);
    static std::vector<tod_gl::RenderableElementComponent> createCharacterRenderables(
        const int maxExpectedCharacters, const std::string& relativeVertexShaderPath,
        const std::string& relativeFragmentShaderPath, const std::string& packagePath);
    static void clearVerticesAndTextures(tod_gl::RenderableElementComponent& renderable);
    static void updateVertices(tod_gl::Mesh& currentMesh, const float tmpAdvance, const tod_gl::Character& currentChar,
                               const float& scale, const float& y, const float& ratioPixelPerMeter);
    static void updateAdvance(float& tmpAdvance, const tod_gl::Character& currentChar, const float& scale);
    static void writeTextIntoRenderable(const std::string& text,
                                        tod_gl::RenderableElementComponent& renderable,
                                        const tod_gl::CharacterMapComponent& characterMap);
    static std::string floatToIntString(const float number);
    static std::string eGearPositionToString(const int gearPosition);
};

}  // namespace TodStaticEntities
