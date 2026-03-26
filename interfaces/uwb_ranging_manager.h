#pragma once

#include <vector>

#include "common/packet.h"

class UwbRangingManagerInterface {
 public:
  virtual ~UwbRangingManagerInterface() = default;
  virtual void onPacketReceived(const ::Packet& pkt) = 0;

  // Returns the latest trilaterated position estimate {x, y, z}.
  // Empty if not enough anchor measurements are available yet.
  virtual std::vector<double> getEstimatedPosition() const = 0;
};
