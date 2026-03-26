#pragma once

namespace sim {

struct UwbChannelConfig {
  // Maximum communication/ranging distance (meters).
  double maxRangeMeters = 50.0;

  // Speed of light (m/s) — used to compute propagation delay.
  double speedOfLightMps = 299792458.0;
};

}  // namespace sim
