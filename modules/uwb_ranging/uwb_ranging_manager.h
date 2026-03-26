#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "interfaces/uwb_ranging_manager.h"
#include "common/messages.h"
#include "common/vector3D.h"

// Processes UWB_BEACON packets to compute ToF-based range measurements,
// then trilaterates the drone's position from 3+ anchor ranges.
//
// Follows the same pattern as FloodManager / NeighborManager:
// - DispatchManager routes UWB_BEACON packets here via onPacketReceived()
// - getEstimatedPosition() returns the latest trilaterated coordinates
class UwbRangingManager : public UwbRangingManagerInterface {
 public:
  // get_time_s: returns current simulation time in seconds (injected to stay platform-agnostic).
  // speed_of_light_mps: must match UwbChannelConfig::speedOfLightMps.
  UwbRangingManager(std::function<double()> get_time_s,
                    double speed_of_light_mps = 299792458.0);

  void onPacketReceived(const ::Packet& pkt) override;

  // Returns {x, y, z} from latest trilateration, or empty if < 3 anchors.
  std::vector<double> getEstimatedPosition() const override;

 private:
  struct AnchorMeasurement {
    uint8_t anchor_id;
    double  range_m;       // computed from ToF
    double  anchor_x;
    double  anchor_y;
    double  anchor_z;
    double  last_seen_s;   // simulation time when measurement was taken
  };

  // Trilaterate from 3+ anchor measurements.
  // Uses linearized least-squares (subtract first anchor equation from the rest).
  bool trilaterate();

  void expireStale();

  std::function<double()> m_get_time_s;
  double m_speed_of_light_mps;
  double m_stale_threshold_s = 0.5;  // drop anchors not heard for this long

  // Latest range measurement per anchor (overwritten each beacon).
  std::unordered_map<uint8_t, AnchorMeasurement> m_measurements;

  // Cached trilateration result.
  bool m_has_position = false;
  double m_est_x = 0.0;
  double m_est_y = 0.0;
  double m_est_z = 0.0;
};
