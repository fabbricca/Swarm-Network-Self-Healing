#pragma once

#include <vector>

#include "interfaces/position.h"
#include "interfaces/uwb_ranging_manager.h"

// PositionInterface adapter that sources coordinates from UWB trilateration.
//
// retrieveCurrentPosition() is a no-op (trilateration runs asynchronously
// on beacon arrival). getCoordinates() returns the latest trilaterated result.
class UwbPosition : public PositionInterface {
 public:
  explicit UwbPosition(UwbRangingManagerInterface* ranging_manager)
      : m_ranging(ranging_manager) {}

  void retrieveCurrentPosition() override {
    if (!m_ranging) {
      return;
    }
    const auto est = m_ranging->getEstimatedPosition();
    if (est.size() >= 3) {
      m_x = est[0];
      m_y = est[1];
      m_z = est[2];
      m_valid = true;
    }
  }

  std::vector<double> getCoordinates() const override {
    return {m_x, m_y, m_z};
  }

  Vector3D distanceFrom(const PositionInterface* other) const override {
    const auto other_coords = other->getCoordinates();
    return Vector3D(other_coords[0] - m_x, other_coords[1] - m_y, other_coords[2] - m_z);
  }

  Vector3D distanceFromCoords(const std::vector<double>& other_coords) const override {
    return Vector3D(other_coords[0] - m_x, other_coords[1] - m_y, other_coords[2] - m_z);
  }

  bool hasValidPosition() const { return m_valid; }

 private:
  UwbRangingManagerInterface* m_ranging;
  double m_x = 0.0;
  double m_y = 0.0;
  double m_z = 0.0;
  bool m_valid = false;
};
