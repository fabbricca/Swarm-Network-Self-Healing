#include "platform/ns3/base_station/ns3_base_station.h"

Ns3BaseStation::Ns3BaseStation(
  uint8_t id,
  ::ns3::Ptr<::ns3::Node> node
) :
  Ns3UwbAnchor(id, node)
{
  m_comm.setReceiveHandler([this](const ::Packet& pkt) { dispatchPacket(pkt); });
  m_dispatcher.setFallbackHandler([this](const ::Packet& pkt) { handleCorePacket(pkt); });
}

void Ns3BaseStation::start() {
  // Start beacon broadcasting (inherited).
  Ns3UwbAnchor::start();

  // Start flood triggering.
  double initial_delay_s = 0.1;
  ::ns3::Simulator::Schedule(::ns3::Seconds(initial_delay_s), ::ns3::MakeCallback(&Ns3BaseStation::onTick, this));
}

void Ns3BaseStation::onTick() {
  if (!m_drone_ids.empty()) {
    // Choose a stable initiator: the lowest registered drone id.
    uint8_t initiator = 0;
    for (const auto& kv : m_drone_ids) {
      initiator = (initiator == 0) ? kv.first : static_cast<uint8_t>(std::min<int>(initiator, kv.first));
    }
    if (initiator != 0) {
      requestFlood(++m_flood_seq, initiator);
    }
  }

  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_dt_s), ::ns3::MakeCallback(&Ns3BaseStation::onTick, this));
}

void Ns3BaseStation::registerDrone(uint8_t id) {
  m_drone_ids[id] = true;
  m_comm.registerPeer(id, 0);  // address unused with UWB transport
}

void Ns3BaseStation::requestFlood(uint16_t flood_id, uint8_t initiator_drone_id) {
  FloodStartMsg msg;
  msg.flood_id = flood_id;

  ::Packet out;
  out.type = ::PacketType::FLOOD;
  out.src = m_id;
  out.dst = initiator_drone_id;
  out.payload.resize(sizeof(msg));
  std::memcpy(out.payload.data(), &msg, sizeof(msg));

  m_comm.send(out);
}

void Ns3BaseStation::dispatchPacket(const ::Packet& pkt) {
  if (pkt.payload.empty()) {
    return;
  }

  if (pkt.dst != m_id && pkt.dst != BROADCAST_ID) {
    return;
  }

  m_dispatcher.handlePacket(pkt);
}

void Ns3BaseStation::handleCorePacket(const ::Packet& pkt) {
  if (pkt.payload.size() < 1) {
    return;
  }

  const auto type = static_cast<SimMsgType>(pkt.payload[0]);
  switch (type) {
    case SimMsgType::POS_UPDATE: {
      if (pkt.payload.size() < sizeof(PositionUpdateMsg)) {
        return;
      }
      PositionUpdateMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));
      handlePositionUpdate(msg, pkt.src);
      return;
    }

    case SimMsgType::HELP_PROXY:
    case SimMsgType::POS_ACK:
    default:
      return;
  }
}

void Ns3BaseStation::handlePositionUpdate(const PositionUpdateMsg& msg, uint8_t relay_src) {
  if (msg.base_id != m_id) {
    return;
  }

  m_last_position[msg.drone_id] = msg;

  sendPositionAck(msg.drone_id, msg.seq, relay_src);
}

void Ns3BaseStation::sendPositionAck(uint8_t drone_id, uint16_t seq, uint8_t relay_src) {
  if (m_position) {
    m_position->retrieveCurrentPosition();
  }
  const auto coords = m_position ? m_position->getCoordinates() : std::vector<double>{};

  PositionAckMsg ack;
  ack.base_id = m_id;
  ack.drone_id = drone_id;
  ack.seq = seq;
  ack.base_hops_to_base_station = 0;
  ack.x = coords.size() > 0 ? coords[0] : 0.0;
  ack.y = coords.size() > 1 ? coords[1] : 0.0;
  ack.z = coords.size() > 2 ? coords[2] : 0.0;

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = relay_src;
  out.payload.resize(sizeof(ack));
  std::memcpy(out.payload.data(), &ack, sizeof(ack));

  m_comm.send(out);
}
