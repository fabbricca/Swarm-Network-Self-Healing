#include "platform/ns3/uwb_transport/uwb_transport.h"

namespace sim {

UwbTransport::UwbTransport(::ns3::Ptr<::ns3::Node> node, uint8_t id) : m_id(id) {
  UwbChannel::Get().Register(id, node, [this](const std::vector<uint8_t>& bytes) {
    if (m_rx_cb) {
      m_rx_cb(bytes);
    }
  });
}

void UwbTransport::RegisterPeer(uint8_t /*id*/, uint32_t /*address*/) {
  // No-op: UwbChannel routes by node ID, no address mapping needed.
}

void UwbTransport::SendUnicast(uint8_t dst_id, const Bytes& bytes) {
  UwbChannel::Get().TransmitTo(m_id, dst_id, bytes);
}

void UwbTransport::SendBroadcast(const Bytes& bytes) {
  UwbChannel::Get().Transmit(m_id, bytes);
}

void UwbTransport::SetRxCallback(RxCallback cb) {
  m_rx_cb = std::move(cb);
}

}  // namespace sim
