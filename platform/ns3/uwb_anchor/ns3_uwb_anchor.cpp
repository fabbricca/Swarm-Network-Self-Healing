#include "platform/ns3/uwb_anchor/ns3_uwb_anchor.h"

#include <cstring>

Ns3UwbAnchor::Ns3UwbAnchor(
  uint8_t id,
  ::ns3::Ptr<::ns3::Node> node
) :
  m_id(id),
  m_node(node),
  m_comm(std::make_unique<sim::UwbTransport>(node, id), id)
{
  if (!m_node) {
    return;
  }

  auto mobility = m_node->GetObject<::ns3::ConstantPositionMobilityModel>();
  if (!mobility) {
    mobility = ::ns3::CreateObject<::ns3::ConstantPositionMobilityModel>();
    m_node->AggregateObject(mobility);
  }

  m_custom_mobility = std::make_unique<CustomMobility>(mobility);
  m_position = std::make_unique<Ns3Position>(m_custom_mobility.get());
}

void Ns3UwbAnchor::start() {
  double initial_delay_s = 0.05;
  ::ns3::Simulator::Schedule(
    ::ns3::Seconds(initial_delay_s),
    ::ns3::MakeCallback(&Ns3UwbAnchor::onBeaconTick, this));
}

void Ns3UwbAnchor::onBeaconTick() {
  sendBeacon();

  ::ns3::Simulator::Schedule(
    ::ns3::Seconds(m_beacon_dt_s),
    ::ns3::MakeCallback(&Ns3UwbAnchor::onBeaconTick, this));
}

void Ns3UwbAnchor::setPosition(double x, double y, double z) {
  if (!m_custom_mobility) {
    return;
  }
  m_custom_mobility->setPosition(x, y, z);
  if (m_position) {
    m_position->retrieveCurrentPosition();
  }
}

void Ns3UwbAnchor::sendBeacon() {
  if (m_position) {
    m_position->retrieveCurrentPosition();
  }
  const auto coords = m_position ? m_position->getCoordinates() : std::vector<double>{};

  UwbBeaconMsg beacon;
  beacon.anchor_id = m_id;
  beacon.tx_timestamp_s = ::ns3::Simulator::Now().GetSeconds();
  beacon.x = coords.size() > 0 ? coords[0] : 0.0;
  beacon.y = coords.size() > 1 ? coords[1] : 0.0;
  beacon.z = coords.size() > 2 ? coords[2] : 0.0;

  ::Packet out;
  out.type = ::PacketType::UWB_BEACON;
  out.src = m_id;
  out.dst = BROADCAST_ID;
  out.payload.resize(sizeof(beacon));
  std::memcpy(out.payload.data(), &beacon, sizeof(beacon));

  m_comm.send(out);
}
