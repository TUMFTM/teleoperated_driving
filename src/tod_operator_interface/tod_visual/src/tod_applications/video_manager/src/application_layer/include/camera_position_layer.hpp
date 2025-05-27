/**
 * @file camera_position_layer.hpp
 * @brief Header file for the CameraPositionLayer class, managing the rendering of camera segments in the UI.
 *
 * @copyright 2024 TUMFTM
 */

#ifndef CAMERA_POSITION_LAYER_HPP
#define CAMERA_POSITION_LAYER_HPP

#include "imgui/imgui.h"
#include <vector>
#include <cmath>
#include <iostream>
#include "tod_gl/layers/imgui_layer.hpp"
#include "stb_image/stb_image.h"
#include <GL/gl.h>  
#include "shared_cam.hpp"
#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"
#include "video_manager_docking_layer.hpp"

/**
 * @struct Segment
 * @brief Represents a segment in the camera position UI.
 */
struct Segment {
    ImVec2 p1;    ///< Starting point of the segment.
    ImVec2 p2;    ///< Ending point of the segment.
    ImVec2 center; ///< Center point of the segment.
    int id;       ///< Unique identifier for the segment.
};

/**
 * @brief Calculates a point on an ellipse for a given angle.
 * 
 * @param cx Center x-coordinate of the ellipse.
 * @param cy Center y-coordinate of the ellipse.
 * @param rx Radius in the x direction.
 * @param ry Radius in the y direction.
 * @param angle Angle in radians for the point.
 * @return ImVec2 The calculated point on the ellipse.
 */
ImVec2 CalculateEllipsePoint(float cx, float cy, float rx, float ry, float angle);

/**
 * @class CameraPositionLayer
 * @brief Manages the rendering and interaction of camera position segments in the UI.
 */
class CameraPositionLayer : public tod_gl::VideoManagerDockingLayer {
public:
    /**
     * @brief Constructs a CameraPositionLayer object.
     * 
     * @param ros Shared pointer to the ROS interface.
     * @param cams Shared pointer to a vector of SharedCam objects.
     */
    CameraPositionLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<std::vector<std::shared_ptr<SharedCam>>> cams, ImGuiDir split_dir);

    /**
     * @brief Called when the layer is attached to the application.
     */
    virtual void on_attach() override;

    /**
     * @brief Called when the layer is detached from the application.
     */
    virtual void on_detach() override;

    /**
     * @brief Handles the rendering of the ImGui components for the layer.
     */
    virtual void on_im_gui_render() override;


    /**
     * @brief Initializes the segments for the UI.
     */
    void initialize_segments();

    /**
     * @brief Determines if a point is within a polygon.
     * 
     * @param point Point to check.
     * @param polygon Polygon to check against.
     * @return true If the point is within the polygon.
     * @return false Otherwise.
     */
    bool is_point_in_polygon(const ImVec2& point, const std::vector<ImVec2>& polygon);


    /**
     * @brief Renders buttons for missing cameras.
     */
    void render_buttons_missing_cams();

private:
    std::vector<Segment> _segments; ///< List of segments to render.
    std::shared_ptr<std::vector<std::shared_ptr<SharedCam>>> _shared_cams_list; ///< Shared list of camera objects.
    tod_gl::TodStatusComponent _status; ///< Status component for TOD interface.
};

#endif // CAMERA_POSITION_LAYER_HPP
