#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include "platform/ns3/uwb_channel/uwb_channel_config.h"

namespace sim {

// Direct PHY-level UWB channel — no MAC, no IP.
//
// Replaces the WiFi ad-hoc stack with a simple propagation model:
// - TX at Simulator::Now() is the PHY TX time (no MAC queuing/backoff)
// - RX is scheduled at TX_time + distance / speed_of_light
// - Packets beyond maxRangeMeters are dropped
//
// This gives exact ToF timestamps for TDoA-based trilateration.
class UwbChannel {
 public:
  using RxCallback = std::function<void(const std::vector<uint8_t>&)>;

  static UwbChannel& Get();

  // Must be called before the first Register().
  void Configure(const UwbChannelConfig& cfg);

  // Register a node on the channel. The rx_callback fires for each received frame.
  void Register(uint8_t id, ::ns3::Ptr<::ns3::Node> node, RxCallback rx_callback);

  // Broadcast: deliver to all registered nodes in range (except sender).
  void Transmit(uint8_t src_id, const std::vector<uint8_t>& bytes);

  // Unicast: deliver to a specific node if in range.
  void TransmitTo(uint8_t src_id, uint8_t dst_id, const std::vector<uint8_t>& bytes);

  double MaxRange() const { return m_cfg.maxRangeMeters; }

 private:
  UwbChannel();

  struct Endpoint {
    uint8_t id;
    ::ns3::Ptr<::ns3::Node> node;
    RxCallback rx_callback;
  };

  // Deliver a frame to a destination endpoint after propagation delay.
  void ScheduleDelivery(const Endpoint& src, const Endpoint& dst,
                        std::shared_ptr<std::vector<uint8_t>> bytes);

  // Called by NS-3 scheduler at the correct receive time.
  void Deliver(uint8_t dst_id, std::shared_ptr<std::vector<uint8_t>> bytes);

  UwbChannelConfig m_cfg;
  std::unordered_map<uint8_t, Endpoint> m_endpoints;
};

}  // namespace sim
