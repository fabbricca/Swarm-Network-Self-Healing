#include "platform/ns3/uwb_channel/uwb_channel.h"

namespace sim {

UwbChannel& UwbChannel::Get() {
  static UwbChannel instance;
  return instance;
}

UwbChannel::UwbChannel() = default;

void UwbChannel::Configure(const UwbChannelConfig& cfg) {
  m_cfg = cfg;
}

void UwbChannel::Register(uint8_t id, ::ns3::Ptr<::ns3::Node> node, RxCallback rx_callback) {
  Endpoint ep;
  ep.id = id;
  ep.node = node;
  ep.rx_callback = std::move(rx_callback);
  m_endpoints[id] = std::move(ep);
}

void UwbChannel::Transmit(uint8_t src_id, const std::vector<uint8_t>& bytes) {
  auto src_it = m_endpoints.find(src_id);
  if (src_it == m_endpoints.end()) {
    return;
  }

  auto bytes_copy = std::make_shared<std::vector<uint8_t>>(bytes);

  for (const auto& [id, ep] : m_endpoints) {
    if (id == src_id) {
      continue;
    }
    ScheduleDelivery(src_it->second, ep, bytes_copy);
  }
}

void UwbChannel::TransmitTo(uint8_t src_id, uint8_t dst_id, const std::vector<uint8_t>& bytes) {
  auto src_it = m_endpoints.find(src_id);
  auto dst_it = m_endpoints.find(dst_id);
  if (src_it == m_endpoints.end() || dst_it == m_endpoints.end()) {
    return;
  }

  auto bytes_copy = std::make_shared<std::vector<uint8_t>>(bytes);
  ScheduleDelivery(src_it->second, dst_it->second, bytes_copy);
}

void UwbChannel::ScheduleDelivery(const Endpoint& src, const Endpoint& dst,
                                  std::shared_ptr<std::vector<uint8_t>> bytes) {
  auto src_mob = src.node->GetObject<::ns3::MobilityModel>();
  auto dst_mob = dst.node->GetObject<::ns3::MobilityModel>();
  if (!src_mob || !dst_mob) {
    return;
  }

  double dist = ::ns3::CalculateDistance(src_mob->GetPosition(), dst_mob->GetPosition());
  if (dist > m_cfg.maxRangeMeters) {
    return;
  }

  double delay_s = dist / m_cfg.speedOfLightMps;

  ::ns3::Simulator::Schedule(
    ::ns3::Seconds(delay_s),
    &UwbChannel::Deliver, this, dst.id, bytes);
}

void UwbChannel::Deliver(uint8_t dst_id, std::shared_ptr<std::vector<uint8_t>> bytes) {
  auto it = m_endpoints.find(dst_id);
  if (it != m_endpoints.end() && it->second.rx_callback) {
    it->second.rx_callback(*bytes);
  }
}

}  // namespace sim
