/**
 * @file lanelet_mgrs_projector.hpp
 * @brief Proper projection of our lanelet into our coordinate system
 * @copyright 2024 TUMFTM
**/

#pragma once

#include <lanelet2_io/Projection.h>

namespace lanelet::projection {

class MGRSProjector : public Projector {
  public:
    explicit MGRSProjector(Origin origin);
    BasicPoint3d forward(const GPSPoint& gps) const override;
    GPSPoint reverse(const BasicPoint3d& mgrs) const override;

  private:
    // Mutable necessary since forward and reverse need to be const
    mutable int zone_{1};
    mutable int k_x_{0};
    mutable int k_y_{0};
    mutable bool northp_{true};
};

}  // namespace lanelet::projection
