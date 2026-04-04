#pragma once

#include <cstdint>
#include <memory>

#include "interfaces/position.h"

#include "modules/communication/communication_manager.h"

#include "common/messages.h"
#include "common/packet.h"

#include "ns3/core-module.h"
#include "ns3/constant-position-mobility-model.h"

#include "platform/ns3/custom_mobility/custom_mobility.h"
#include "platform/ns3/position/ns3_position.h"
#include "platform/ns3/uwb_transport/uwb_transport.h"

// UWB anchor node: static-position beacon broadcaster.
//
// Periodically broadcasts UwbBeaconMsg containing its ID, position,
// and a simulation timestamp so that drones can compute ToF-based
// range estimates for trilateration.
//
// Ns3BaseStation derives from this class, adding drone communication
// and flood-triggering on top of the anchor beacon behavior.
class Ns3UwbAnchor {
 public:
  Ns3UwbAnchor(uint8_t id, ::ns3::Ptr<::ns3::Node> node);
  virtual ~Ns3UwbAnchor() = default;

  uint8_t id() const { return m_id; }

  PositionInterface* position() const { return m_position.get(); }

  void setPosition(double x, double y, double z);

  // Starts periodic beacon broadcasting.
  virtual void start();

 protected:
  // Broadcast a single UWB beacon with current position and timestamp.
  void sendBeacon();

  // Override in subclasses to schedule additional periodic work.
  virtual void onBeaconTick();

  uint8_t m_id;
  ::ns3::Ptr<::ns3::Node> m_node;

  std::unique_ptr<CustomMobility> m_custom_mobility;
  std::unique_ptr<Ns3Position> m_position;

  CommunicationManager m_comm;

  double m_beacon_dt_s = 0.1;  // 10 Hz beacon rate (sufficient for trilateration; 100 Hz overloads NS-3 at scale)
};
