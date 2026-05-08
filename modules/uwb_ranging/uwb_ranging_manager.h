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
                    double speed_of_light_mps = 299792458.0,
                    double noise_std_dev_m = 0.0);

  void onPacketReceived(const ::Packet& pkt) override;

  // Returns {x, y, z} from latest trilateration, or empty if < 3 anchors.
  std::vector<double> getEstimatedPosition() const override;

  // Seed the cached estimate with a known initial position (e.g. from an
  // onboard barometer/GPS snapshot at boot).  Without seeding, 2D trilat
  // leaves z at 0 while the drone physically operates at ~20 m, and drones
  // that never achieve a UWB fix remain at (0,0,0) forever.
  void seedEstimatedPosition(double x, double y, double z);

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
  double m_noise_std_dev_m;
  // Drop anchors we haven't heard from for this long.  Lifted from 0.5 s to 1.5 s
  // when anchor beacons moved to 2 Hz — we still want to prune truly-out-of-range
  // anchors but must not expire the current ones between consecutive beacons.
  double m_stale_threshold_s = 1.5;

  // Latest range measurement per anchor (overwritten each beacon).
  std::unordered_map<uint8_t, AnchorMeasurement> m_measurements;

  // Cached trilateration result.
  bool m_has_position = false;
  double m_est_x = 0.0;
  double m_est_y = 0.0;
  double m_est_z = 0.0;

  // Anchor z-spread required to attempt 3D trilateration.  A handful of
  // anchors scattered over z ∈ [2, 5] m is geometrically coplanar from a
  // drone at ~20 m altitude — vertical GDOP is catastrophic and the solver
  // returns garbage that drives the controller into phantom-altitude chases.
  // Below this threshold, fall back to 2D (xy only) and preserve the previous
  // z estimate so the controller sees a stable altitude.
  static constexpr double Z_SPREAD_MIN_FOR_3D_M = 5.0;
};
