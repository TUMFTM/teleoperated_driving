/**
 * @file traffic_sign_layer.hpp
 * @brief Provides a layer that displays relevant traffic signs based on the vehicle's current position and lanelet map data.
 *        Updates the visible traffic sign in real-time according to the nearest lanelet regulatory elements.
 * @copyright 2024 TUMFTM
 */

/**
 * @class TrafficSignLayer
 * @brief Displays a traffic sign overlay determined by the vehicle's position and a loaded lanelet map.
 * @tparam KinematicStateComp Component type providing the vehicle's position information.
 */
#pragma once

#include "ui_layer.hpp"
#include "view_port_layer.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/primitives/RegulatoryElement.h"
#include "lanelet2_io/Io.h"
#include "lanelet2_routing/RoutingGraph.h"

#include "tod_gl/ros_interface/subscribing_components/odometry_component.hpp"

#include "tod_dynamic_entities/lanelet_map_renderer.hpp"

namespace tod_visual {
template <class KinematicStateComp>
class TrafficSignLayer : public UILayer {
  public:
      /**
     * @brief Constructs the TrafficSignLayer.
     * @param ros Shared pointer to the ROS interface.
     * @param scene Shared pointer to the scene.
     * @param view_port_layer Pointer to the associated ViewPortLayer.
     */
    TrafficSignLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene,
                     ViewPortLayer *view_port_layer)
        : UILayer(ros, scene, view_port_layer) {
        _name = "TrafficSignLayer";
        auto laneletRendererEntity = scene->find_entity_with_tag("Lanelet");
        laneletMap = laneletRendererEntity.;
        // LaneletMapRenderer &laneletMapRenderer = LaneletMapRenderer::get_instance();
        // laneletMap = laneletMapRenderer.LoadMap(49.00513434186, 8.41520892443, 519.54433485213667,
        // tod_gl::RosInterface::get_package_path() + "/resources/maps/mapping_example.osm");
    }
    ~TrafficSignLayer() = default;

    virtual void on_im_gui_render() override {
        ImGuiIO &io = ImGui::GetIO();
        ImVec2 currentDisplaySize = io.DisplaySize;
        ImVec2 currentWindowPos = ImGui::GetWindowPos();
        ImVec2 viewportPosition = view_port_layer->getPos();
        ImVec2 viewportSize = view_port_layer->getSize();

        ImGuiViewport *mainViewport = ImGui::GetMainViewport();
        ImVec2 mainViewportPos = mainViewport->Pos;

        float accountForSizeDifference =
            (currentDisplaySize.y * 0.17f > 320.0f) ? (currentDisplaySize.y * 0.17f - 320.0f) : 0.0f;
        ImVec2 trafficSignPos = ImVec2(
            mainViewportPos.x + currentDisplaySize.x * 0.5f - signSize.x * 0.7f,
            mainViewportPos.y + currentDisplaySize.y * 0.87f - signSize.y * 1.1f + accountForSizeDifference + 20);

        ImGui::SetNextWindowPos(trafficSignPos);
        ImGui::SetNextWindowSize(ImVec2(signSize.x * 1.5f, signSize.y * 1.5f));

        ImGui::Begin("TrafficSign", nullptr, /*ImGuiWindowFlags_NoInputs | */ ImGuiWindowFlags_NoMove |
                                                 ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoCollapse |
                                                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                                                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar);

        RenderTrafficSign();

        ImGui::End();
    }

    virtual void on_attach() override {}
    virtual void on_event(tod_gl::Event &e) override { ImGuiSceneLayer::on_event(e); }

    virtual void on_update(float ts) override {
        tod_gl::Entity SubscriptionManager = _active_scene->find_entity_with_tag("SubscriptionManager");
        if (SubscriptionManager.has_component<KinematicStateComp>()) {
            KinematicStateComp &comp = SubscriptionManager.get_component<KinematicStateComp>();
            Position = comp.position;
            UpdateTrafficSign(Position);
        }
    }

    void RenderTrafficSign() {
        if (!currentTrafficSign.empty()) {
            std::string texturePath =
                tod_gl::RosInterface::get_package_path() + "/resources/traffic_signs/" + currentTrafficSign + ".png";
            try {
                GLuint textureId = load_texture(texturePath.c_str());
                ImGui::Image((void *)(intptr_t)textureId, signSize);
            } catch (const std::runtime_error &e) {
                std::cerr << "Failed to load texture: " << e.what() << std::endl;
            }
        }
    }

  private:
    glm::vec3 Position = glm::vec3(0, 0, 0);
    std::string currentTrafficSign;
    lanelet::LaneletMapConstPtr laneletMap;
    ImVec2 signSize = ImVec2(45.0f, 45.0f);



    /**
     * @brief Determines the relevant traffic sign based on the vehicle's lanelet location and adjusts depending on osm file and names of the traffic signs
     * @param position The current 3D position of the vehicle.
     */
    void UpdateTrafficSign(const glm::vec3 &position) {
        if (!laneletMap) {
            std::cerr << "Lanelet map not loaded" << std::endl;
            return;
        }

        lanelet::BasicPoint2d point(position.x, position.y);
        auto nearestLanelets = lanelet::geometry::findNearest(laneletMap->laneletLayer, point, 1);

        if (!nearestLanelets.empty()) {
            lanelet::ConstLanelet nearestLanelet = nearestLanelets[0].second;

            for (const auto &regElem : nearestLanelet.regulatoryElements()) {
                if (regElem->attribute("subtype").value() == "speed_limit") {
                    currentTrafficSign = regElem->attribute("sign_type").value();
                }
                if (regElem->attribute("subtype").value() == "right_of_way") {
                    currentTrafficSign = "DE/306";
                }
            }
        } else {
            currentTrafficSign.clear();  // Clear if no lanelet is found
        }
    }
};
}  // namespace tod_visual