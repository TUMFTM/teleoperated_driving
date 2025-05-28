/**
 * @file lanelet_mgrs_projector.cpp
 * @brief Proper projection of our lanelet into our coordinate system
 * @copyright 2024 TUMFTM
**/

#include "tod_dynamic_entities/utils/lanelet_mgrs_projector.hpp"

#include <lanelet2_io/Exceptions.h>
#include <GeographicLib/UTMUPS.hpp>

namespace lanelet::projection {

MGRSProjector::MGRSProjector(Origin origin) : Projector(origin) {}

BasicPoint3d MGRSProjector::forward(const GPSPoint& gps) const {
    BasicPoint3d mgrs{0., 0., gps.ele};
    // Convert from GPS to UTM
    GeographicLib::UTMUPS::Forward(gps.lat, gps.lon, zone_, northp_, mgrs.x(), mgrs.y());

    // Workaround for reverse conversion
    k_x_ = static_cast<int>(mgrs.x() / 100000.0);
    k_y_ = static_cast<int>(mgrs.y() / 100000.0);

    // Convert from UTM to MGRS
    mgrs.x() = std::fmod(mgrs.x(), 100000.0);
    mgrs.y() = std::fmod(mgrs.y(), 100000.0);

    return mgrs;
}

GPSPoint MGRSProjector::reverse(const BasicPoint3d& mgrs) const {
    GPSPoint gps{0., 0., mgrs.z()};

    GeographicLib::UTMUPS::Reverse(zone_, northp_, k_x_ * mgrs.x(), k_y_ * mgrs.y(), gps.lat, gps.lon);
    return gps;
}

}  // namespace lanelet::projection
