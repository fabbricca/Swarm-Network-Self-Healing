#pragma once

#include <functional>

#include "common/packet.h"
#include "interfaces/flood_manager.h"
#include "interfaces/neighbor_manager.h"
#include "interfaces/uwb_ranging_manager.h"

class DispatchManagerInterface {
 public:
  using FallbackHandler = std::function<void(const ::Packet&)>;

  virtual ~DispatchManagerInterface() = default;

  virtual void setFloodManager(FloodManagerInterface* flood_manager) = 0;
  virtual void setNeighborManager(NeighborManagerInterface* neighbor_manager) = 0;
  virtual void setUwbRangingManager(UwbRangingManagerInterface* uwb_ranging_manager) = 0;
  virtual void setFallbackHandler(FallbackHandler handler) = 0;

  virtual void handlePacket(const ::Packet& pkt) const = 0;
};
