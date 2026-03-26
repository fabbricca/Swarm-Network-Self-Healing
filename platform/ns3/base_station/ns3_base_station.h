#pragma once

#include <cstdint>
#include <unordered_map>
#include <cstring>

#include "modules/dispatch/dispatch_manager.h"
#include "modules/flood/flood_messages.h"

#include "common/messages.h"
#include "common/packet.h"

#include "platform/ns3/uwb_anchor/ns3_uwb_anchor.h"

// NS-3 bound base station node logic.
// Derives from Ns3UwbAnchor to also broadcast UWB beacons.
// - Static position.
// - Receives PositionUpdateMsg from drones and replies with PositionAckMsg.
// - Triggers periodic flooding for hop discovery.
class Ns3BaseStation : public Ns3UwbAnchor {
 public:
  Ns3BaseStation(uint8_t id, ::ns3::Ptr<::ns3::Node> node);

  void registerDrone(uint8_t id);

  // Starts beacon broadcasting (inherited) + flood triggering.
  void start() override;

  // Trigger a new flood by unicast start to an initiator drone.
  void requestFlood(uint16_t flood_id, uint8_t initiator_drone_id);

 private:
  void onTick();

  void dispatchPacket(const ::Packet& pkt);
  void handleCorePacket(const ::Packet& pkt);

  void handlePositionUpdate(const PositionUpdateMsg& msg, uint8_t relay_src);

  void sendPositionAck(uint8_t drone_id, uint16_t seq, uint8_t relay_src);

  DispatchManager m_dispatcher;

  std::unordered_map<uint8_t, bool> m_drone_ids;
  std::unordered_map<uint8_t, PositionUpdateMsg> m_last_position;

  double m_tick_dt_s = 0.05;
  uint16_t m_flood_seq = 0;
};
