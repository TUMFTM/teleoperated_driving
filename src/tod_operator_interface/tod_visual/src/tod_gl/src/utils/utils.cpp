/**
 * @file utils.cpp
 * @brief Utils for rendering 
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/utils/utils.hpp"

namespace tod_gl {


/**
 * @brief Triangulates vertices for a line segment.
 * 
 * This function computes the vertices required to render a thick line by
 * extruding a given position vector along a perpendicular direction.
 * It then updates the provided mesh with the calculated vertices and indices.
 * 
 * @param posVec Position vector of the line.
 * @param right Direction perpendicular to the line for width computation.
 * @param color Color of the line.
 * @param mesh Mesh structure to store the generated vertices and indices.
 * @param lineWidth Thickness of the line.
 */
void Utils::triangulate_for_line(glm::vec3& posVec, glm::vec3& right, glm::vec3& color, Mesh& mesh, float lineWidth) {
    const glm::vec3 moveVec = right * (lineWidth / 2.f);
    mesh.vertices.push_back(Vertex(posVec + moveVec, glm::vec2(0.0f, 0.0f), color));
    mesh.vertices.push_back(Vertex(posVec - moveVec, glm::vec2(0.0f, 0.0f), color));

    if (mesh.vertices.size() < 4)
        return;

    unsigned int startIdx = mesh.vertices.size() - 4;
    mesh.indices.push_back(startIdx + 0);
    mesh.indices.push_back(startIdx + 1);
    mesh.indices.push_back(startIdx + 3);
    mesh.indices.push_back(startIdx + 0);
    mesh.indices.push_back(startIdx + 3);
    mesh.indices.push_back(startIdx + 2);
}

/**
 * @brief Renders a path using lines with optional tick marks.
 * 
 * Generates a smooth path using line segments and adds perpendicular
 * tick marks at regular intervals. The path color and side highlights
 * are customizable.
 * 
 * @param pathPoints List of 3D points defining the path.
 * @param pathWidth Width of the path.
 * @param colors List of colors corresponding to each segment.
 * @param tickLength Length of the tick marks.
 * @param tickSpacing Distance between tick marks.
 * @param mesh Mesh structure to store the generated geometry.
 * @param sideColor Colors for the side highlights of the path.
 */
void Utils::render_path_lines(
    const std::vector<glm::vec3>& pathPoints,
    float pathWidth,
    const std::vector<glm::vec3>& colors,
    const float tickLength,
    const float tickSpacing,
    Mesh& mesh,
    const std::vector<glm::vec3> sideColor)
{
    if (pathPoints.size() < 2) return;

    glm::vec3 offset = pathPoints[0];
    const glm::vec3 up(0, 0, 1);
    const float halfWidth = pathWidth * 0.5f;
    const float lineWidth = 0.05f; 
    const int smoothingSteps = 50; 

    auto addQuad = [&](const glm::vec3& start, const glm::vec3& end, const glm::vec3& quad_offset,const glm::vec3& color) {
        glm::vec3 direction = glm::normalize(end - start);
        glm::vec3 perpendicular = glm::normalize(glm::cross(direction, up)) * lineWidth;

        glm::vec3 v0 = start + quad_offset - perpendicular + offset;
        glm::vec3 v1 = start + quad_offset + perpendicular + offset;
        glm::vec3 v2 = end + quad_offset - perpendicular + offset;
        glm::vec3 v3 = end + quad_offset + perpendicular + offset;

        mesh.vertices.emplace_back(v0, glm::vec2(0), color);
        mesh.vertices.emplace_back(v1, glm::vec2(0), color);
        mesh.vertices.emplace_back(v2, glm::vec2(0), color);

        mesh.vertices.emplace_back(v1, glm::vec2(0), color);
        mesh.vertices.emplace_back(v3, glm::vec2(0), color);
        mesh.vertices.emplace_back(v2, glm::vec2(0), color);
    };

    auto addTick = [&](const glm::vec3& position, const glm::vec3& direction, const glm::vec3& side) {
        glm::vec3 tickStart = position + side;
        glm::vec3 tickEnd = tickStart - side * (tickLength / halfWidth);
        addQuad(tickStart, tickEnd, glm::vec3(0), glm::vec3(1.0f, 1.0f, 1.0f) );
    };

    float accumulatedDistance = 0.0f;
    for (size_t i = 0; i < pathPoints.size() - 1; ++i)
    {
        glm::vec3 current = pathPoints[i] - offset;
        glm::vec3 next = pathPoints[i + 1] - offset;
        glm::vec3 currentColor = colors[i];
        glm::vec3 nextColor = colors[i + 1];


        for (int step = 0; step < smoothingSteps; ++step)
        {
            float t = static_cast<float>(step) / smoothingSteps;
            glm::vec3 interpolatedStart = glm::mix(current, next, t);
            glm::vec3 interpolatedEnd = glm::mix(current, next, (t + 1.0f / smoothingSteps));

            glm::vec3 direction = glm::normalize(interpolatedEnd - interpolatedStart);
            glm::vec3 side = glm::normalize(glm::cross(direction, up)) * halfWidth;
            float segmentLength = glm::distance(interpolatedStart, interpolatedEnd);
            
            glm::vec3 pathColor = (t < 0.5f) ? currentColor : nextColor;

            addQuad(interpolatedStart, interpolatedEnd, -side, sideColor[i]);
            addQuad(interpolatedStart, interpolatedEnd, glm::vec3(0), pathColor);
            addQuad(interpolatedStart, interpolatedEnd, side, sideColor[i]);

            while (accumulatedDistance + segmentLength >= tickSpacing)
            {
                float tickT = (tickSpacing - accumulatedDistance) / segmentLength;
                glm::vec3 tickPosition = glm::mix(interpolatedStart, interpolatedEnd, tickT);
                
                // Left tick
                addTick(tickPosition, direction, -side);
                // Right tick
                addTick(tickPosition, direction, side);

                accumulatedDistance = 0.0f;
            }

            accumulatedDistance += segmentLength;

        }
        
    }
}


/**
 * @brief Adds a clickable quad to the mesh.
 * 
 * Creates a square quad centered at the given position to be used
 * as an interactive element in rendering.
 * 
 * @param mesh Mesh structure to store the generated quad.
 * @param position Center position of the quad.
 * @param size Size of the quad.
 * @param color Color of the quad.
 */
void Utils::add_quad_for_click(Mesh& mesh, const glm::vec3& position, float size, const glm::vec3& color) {
    float halfSize = size / 2.0f;

    // Define the four corners of the quad
    glm::vec3 topLeft = position + glm::vec3(-halfSize, halfSize, 0.0f);
    glm::vec3 topRight = position + glm::vec3(halfSize, halfSize, 0.0f);
    glm::vec3 bottomLeft = position + glm::vec3(-halfSize, -halfSize, 0.0f);
    glm::vec3 bottomRight = position + glm::vec3(halfSize, -halfSize, 0.0f);

    // Add vertices
    uint32_t baseIndex = mesh.vertices.size();
    mesh.vertices.push_back(Vertex(topLeft, glm::vec2(0.0f, 1.0f), color));
    mesh.vertices.push_back(Vertex(topRight, glm::vec2(1.0f, 1.0f), color));
    mesh.vertices.push_back(Vertex(bottomLeft, glm::vec2(0.0f, 0.0f), color));
    mesh.vertices.push_back(Vertex(bottomRight, glm::vec2(1.0f, 0.0f), color));

    // Add indices for two triangles
    mesh.indices.push_back(baseIndex);
    mesh.indices.push_back(baseIndex + 1);
    mesh.indices.push_back(baseIndex + 2);
    mesh.indices.push_back(baseIndex + 1);
    mesh.indices.push_back(baseIndex + 3);
    mesh.indices.push_back(baseIndex + 2);
}
/**
 * @brief Renders multiple paths with line segments.
 * 
 * Generates and renders multiple smooth paths with line segments.
 * Each path is drawn separately with a predefined color.
 * 
 * @param pathPoints List of paths, where each path is a vector of 3D points.
 * @param lineWidth Width of the path lines.
 * @param mesh Mesh structure to store the generated geometry.
 */
void Utils::render_multiple_paths(const std::vector<std::vector<glm::vec3>>& pathPoints,
                               float lineWidth,
                               Mesh& mesh) {
    mesh.vertices.clear();
    
    const glm::vec3 up(0.0f, 0.0f, 1.0f);
    const glm::vec3 color(.0f, (float)102/255 , 1.0f);
    const int smoothingSteps = 10;

    auto addQuad = [&](const glm::vec3& start, const glm::vec3& end) {
        glm::vec3 direction = glm::normalize(end - start);
        glm::vec3 perpendicular = glm::normalize(glm::cross(direction, up)) * lineWidth;

        glm::vec3 v0 = start - perpendicular;
        glm::vec3 v1 = start + perpendicular;
        glm::vec3 v2 = end - perpendicular;
        glm::vec3 v3 = end + perpendicular;

        mesh.vertices.push_back(Vertex(v0, glm::vec2(0), color));
        mesh.vertices.push_back(Vertex(v1, glm::vec2(0), color));
        mesh.vertices.push_back(Vertex(v2, glm::vec2(0), color));
        mesh.vertices.push_back(Vertex(v1, glm::vec2(0), color));
        mesh.vertices.push_back(Vertex(v3, glm::vec2(0), color));
        mesh.vertices.push_back(Vertex(v2, glm::vec2(0), color));
    };

    for (const auto& points : pathPoints) {
        if (points.size() < 2) continue;

        glm::vec3 offset = points[0];

        for (size_t i = 0; i < points.size() - 1; ++i) {
            glm::vec3 current = points[i] - offset;
            glm::vec3 next = points[i + 1] - offset;

            for (int step = 0; step < smoothingSteps; ++step) {
                float t = static_cast<float>(step) / smoothingSteps;
                glm::vec3 interpolatedStart = glm::mix(current, next, t);
                glm::vec3 interpolatedEnd = glm::mix(current, next, (t + 1.0f / smoothingSteps));

                addQuad(interpolatedStart + offset, interpolatedEnd + offset);
            }
        }
    }
}
/**
 * @brief Renders a simplified path with lines.
 * 
 * Generates a smooth path using line segments and colors the sides separately.
 * This function is a simplified version of `render_path_lines` without tick marks.
 * 
 * @param pathPoints List of 3D points defining the path.
 * @param pathWidth Width of the path.
 * @param mesh Mesh structure to store the generated geometry.
 * @param sideColor Colors for the side highlights of the path.
 */
void Utils::render_path_lines_simple(
    const std::vector<glm::vec3>& pathPoints,
    float pathWidth,
    Mesh& mesh,
    const std::vector<glm::vec3> sideColor)
{
    if (pathPoints.size() < 2) return;

    glm::vec3 offset = pathPoints[0];
    const glm::vec3 up(0, 0, 1);
    const float halfWidth = pathWidth * 0.5f;
    const float lineWidth = 0.05f; 
    const int smoothingSteps = 50; 

    auto addQuad = [&](const glm::vec3& start, const glm::vec3& end, const glm::vec3& quad_offset,const glm::vec3& color) {
        glm::vec3 direction = glm::normalize(end - start);
        glm::vec3 perpendicular = glm::normalize(glm::cross(direction, up)) * lineWidth;

        glm::vec3 v0 = start + quad_offset - perpendicular + offset;
        glm::vec3 v1 = start + quad_offset + perpendicular + offset;
        glm::vec3 v2 = end + quad_offset - perpendicular + offset;
        glm::vec3 v3 = end + quad_offset + perpendicular + offset;

        mesh.vertices.emplace_back(v0, glm::vec2(0), color);
        mesh.vertices.emplace_back(v1, glm::vec2(0), color);
        mesh.vertices.emplace_back(v2, glm::vec2(0), color);

        mesh.vertices.emplace_back(v1, glm::vec2(0), color);
        mesh.vertices.emplace_back(v3, glm::vec2(0), color);
        mesh.vertices.emplace_back(v2, glm::vec2(0), color);
    };

    float accumulatedDistance = 0.0f;
    for (size_t i = 0; i < pathPoints.size() - 1; ++i)
    {
        glm::vec3 current = pathPoints[i] - offset;
        glm::vec3 next = pathPoints[i + 1] - offset;

        for (int step = 0; step < smoothingSteps; ++step)
        {
            float t = static_cast<float>(step) / smoothingSteps;
            glm::vec3 interpolatedStart = glm::mix(current, next, t);
            glm::vec3 interpolatedEnd = glm::mix(current, next, (t + 1.0f / smoothingSteps));

            glm::vec3 direction = glm::normalize(interpolatedEnd - interpolatedStart);
            glm::vec3 side = glm::normalize(glm::cross(direction, up)) * halfWidth;
            

            addQuad(interpolatedStart, interpolatedEnd, -side, sideColor[i]);
            addQuad(interpolatedStart, interpolatedEnd, side, sideColor[i]);

        }
        
    }
}



}  // namespace tod_gl