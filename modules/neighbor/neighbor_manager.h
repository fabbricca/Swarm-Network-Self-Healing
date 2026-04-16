#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <cstring>

#include "interfaces/communication_manager.h"
#include "interfaces/neighbor_manager.h"
#include "interfaces/position.h"

#include "modules/neighbor/neighbor_info.h"

class NeighborManager : public NeighborManagerInterface {
 public:
  explicit NeighborManager(CommunicationManagerInterface* communication_manager);

  void onPacketReceived(const ::Packet& pkt) override;
  std::vector<NeighborInfoInterface*> getNeighbors() const override;
  void sendToNeighbors(uint8_t id, PositionInterface* position, uint8_t hops_to_base_station) override;

 private:
  CommunicationManagerInterface* m_communication_manager;
  std::unordered_map<uint8_t, std::unique_ptr<NeighborInfo>> m_neighbors;

  // Rate-limit NEIGHBOR broadcasts.  The controller invokes sendToNeighbors
  // every physics tick (20 Hz); we count invocations rather than reading a
  // clock so this module stays platform-agnostic.
  //
  // Gate logic (applied once MIN_CALLS_BETWEEN_SENDS has passed):
  //   - skip if the drone hasn't moved more than NEIGHBOR_DELTA_THRESHOLD_M,
  //     UNLESS MAX_CALLS_BETWEEN_SENDS has elapsed (heartbeat fallback so
  //     neighbors don't let this entry time out while we sit still).
  //
  // At 20 Hz: min=5 calls (250 ms), max=40 calls (2 s).
  uint32_t m_call_count = 0;
  uint32_t m_last_sent_call = 0;
  bool m_has_last_sent = false;
  std::vector<double> m_last_sent_coords;
  static constexpr uint32_t MIN_CALLS_BETWEEN_SENDS = 5;
  static constexpr uint32_t MAX_CALLS_BETWEEN_SENDS = 40;
  static constexpr double NEIGHBOR_DELTA_THRESHOLD_M = 0.2;
};