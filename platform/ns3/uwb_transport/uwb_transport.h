#pragma once

#include <cstdint>

#include "ns3/core-module.h"

#include "interfaces/transport.h"
#include "platform/ns3/uwb_channel/uwb_channel.h"

namespace sim {

// UWB PHY-level transport — no MAC, no IP.
//
// Implements the Transport interface over UwbChannel.
// SendBroadcast/SendUnicast map directly to UwbChannel::Transmit/TransmitTo.
// RegisterPeer is a no-op (the channel routes by node ID).
class UwbTransport final : public ::Transport {
 public:
  UwbTransport(::ns3::Ptr<::ns3::Node> node, uint8_t id);

  void RegisterPeer(uint8_t id, uint32_t address) override;
  void SendUnicast(uint8_t dst_id, const Bytes& bytes) override;
  void SendBroadcast(const Bytes& bytes) override;
  void SetRxCallback(RxCallback cb) override;

 private:
  uint8_t m_id;
  RxCallback m_rx_cb;
};

}  // namespace sim
