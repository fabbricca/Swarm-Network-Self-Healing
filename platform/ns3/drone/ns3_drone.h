#pragma once

#include <cstdint>
#include <memory>
#include <cstring>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "interfaces/position.h"

#include "modules/communication/communication_manager.h"
#include "interfaces/controller.h"
#include "modules/controller/controller_factory.h"
#include "modules/dispatch/dispatch_manager.h"
#include "modules/flood/flood_manager.h"
#include "modules/neighbor/neighbor_manager.h"
#include "modules/uwb_ranging/uwb_ranging_manager.h"
#include "modules/uwb_ranging/uwb_position.h"

#include "common/messages.h"
#include "common/packet.h"
#include "common/vector3D.h"

#include "ns3/core-module.h"
#include "ns3/constant-position-mobility-model.h"

#include "platform/ns3/custom_mobility/custom_mobility.h"
#include "platform/ns3/velocity_actuator/ns3_velocity_actuator.h"
#include "platform/ns3/uwb_transport/uwb_transport.h"

// Per-drone packet receive counters broken down by message type.
struct DronePacketStats {
  uint32_t pos_update  = 0;
  uint32_t pos_ack     = 0;
  uint32_t help_proxy  = 0;
  uint32_t flood       = 0;
  uint32_t neighbor    = 0;
  uint32_t uwb_beacon  = 0;
};

// v2 per-base session state on each drone.
struct DroneBaseState {
  // Reachability: freshness of the most recent ACK from this base.  Any
  // ACK (direct or relayed) refreshes last_ack_rx_s; only direct ACKs
  // (pkt.src == ack.base_id) refresh last_direct_ack_rx_s.
  double last_ack_rx_s = 0.0;
  double last_direct_ack_rx_s = -1.0;

  // POS_UPDATE / ACK bookkeeping for this base.  Delta-gate state is
  // per-base because the set of "last-sent" coordinates is relative to
  // the destination (we re-send on base change even without motion).
  uint16_t last_acked_seq = 0;
  bool waiting_ack = false;
  double last_pos_send_s = 0.0;
  double last_sent_x = 0.0, last_sent_y = 0.0, last_sent_z = 0.0;
  bool has_last_sent_pos = false;
};

// NS-3 bound drone node logic.
// - Every drone knows about every base at construction (v2 roaming).
// - Each tick it picks the nearest reachable base and directs its
//   POS_UPDATE / HELP_PROXY / controller logic at that base.
// - When a drone loses its primary base (base killed, or moved out of
//   range), it migrates to the next-nearest reachable base naturally.
class Ns3Drone {
 public:
  Ns3Drone(
    uint8_t id,
    ::ns3::Ptr<::ns3::Node> node,
    ControllerAlgorithm algorithm = ControllerAlgorithm::Centroid,
    float k_att = 1.5f,
    float k_rep = 5.0f,
    float d_safe = 1.0f,
    float v_max = 2.5f,
    float drone_weight_kg = 0.029f,
    double uwb_noise_std_dev_m = 0.0
  );

  uint8_t id() const { return m_id; }

  PositionInterface* position() const { return m_uwb_position.get(); }

  // v2: register every base this drone should track.  Called once per base
  // during setup.  The order of registration seeds the round-robin used
  // for HELP_PROXY.
  void registerBase(uint8_t base_id);

  void startMission();
  void stopMission();

  void start();

  void setRepositionLogger(const std::shared_ptr<std::ofstream>& csv);

  const DronePacketStats& rxStats() const { return m_rx_stats; }

  double totalDistanceTraveled() const { return m_total_distance_m; }

  // Hops reported relative to whichever base is currently nearest (v2).
  uint8_t hopsFromBase() const {
    return m_flood_manager ? m_flood_manager->getNearestBaseHops() : 0xFF;
  }
  uint8_t nearestBaseId() const {
    return m_flood_manager ? m_flood_manager->getNearestBaseId() : 0xFF;
  }

  // Return-behavior telemetry (all negative => event never occurred).
  double helpProxyTxTime() const { return m_last_help_proxy_tx_s; }
  double returnArmedTime() const { return m_return_trigger_time_s; }
  double returnStartTime() const { return m_return_start_s; }
  double returnCompleteTime() const { return m_return_complete_s; }
  bool returnTriggerWasFlag() const { return m_return_trigger_was_flag; }
  bool isReturning() const { return m_returning; }
  bool returnArmed() const { return m_return_triggered; }

  // Metrics telemetry for end-of-sim report.
  double   firstAckAfterHelpS()   const { return m_first_ack_after_help_s; }
  double   returnPhaseDistanceM() const { return m_return_phase_distance_m; }
  uint32_t rearmCount()           const { return m_rearm_count; }
  double   stationKeepingDriftM() const { return m_station_keeping_max_drift_m; }
  Vector3D returnStartPosGt()     const { return m_return_start_pos_gt; }
  Vector3D returnCompletePosGt()  const { return m_return_complete_pos_gt; }
  Vector3D currentGtPos() const {
    return Vector3D(m_prev_gt_x, m_prev_gt_y, m_prev_gt_z);
  }

  // Mid-sim scheduled failure.
  void   kill();
  bool   isAlive() const { return m_alive; }
  double killedAtS() const { return m_killed_at_s; }

 private:
  void onTick();
  void dispatchPacket(const ::Packet& pkt);
  void handleCorePacket(const ::Packet& pkt);

  void sendPositionUpdate();
  void sendHelpProxy();

  // "Reachable" = any known base has a fresh (direct or relayed) ACK.
  bool isBaseReachable() const;
  // "Direct coverage" = any known base has a fresh DIRECT ACK.
  bool hasDirectBaseCoverage() const;
  // Individual base reachability for the FloodManager reachability hook.
  bool isBaseReachableId(uint8_t base_id) const;
  bool hasDirectCoverageId(uint8_t base_id) const;

  uint8_t m_id;
  ::ns3::Ptr<::ns3::Node> m_node;

  std::unique_ptr<CustomMobility> m_custom_mobility;
  std::unique_ptr<Ns3VelocityActuator> m_velocity_actuator;

  // v2: vector for deterministic iteration + round-robin, map for state.
  std::vector<uint8_t> m_known_bases;
  std::unordered_map<uint8_t, DroneBaseState> m_base_state;
  size_t m_help_proxy_target_idx = 0;

  CommunicationManager m_comm;

  std::unique_ptr<FloodManager> m_flood_manager;
  std::unique_ptr<NeighborManager> m_neighbor_manager;
  std::unique_ptr<UwbRangingManager> m_uwb_ranging_manager;
  std::unique_ptr<UwbPosition> m_uwb_position;
  DispatchManager m_dispatcher;

  std::unique_ptr<ControllerInterface> m_controller;

  bool help_proxy_sent = false;

  double m_last_help_proxy_tx_s = -1.0;
  double m_last_help_proxy_rx_s = -1.0;

  double m_mission_start_s = -1.0;
  double m_last_mission_log_s = -1.0;
  double m_mission_log_dt_s = 0.5;

  double m_last_idle_log_s = -1.0;
  double m_idle_log_dt_s = 2.0;

  std::shared_ptr<std::ofstream> m_reposition_csv;

  double m_tick_dt_s = 0.05;
  double m_tick_phase_s = 0.0;

  double m_ack_timeout_s = 1.5;
  static constexpr double DIRECT_ACK_TIMEOUT_S = 1.5;

  bool m_station_keeping = false;

  uint16_t m_pos_seq = 0;
  double m_pos_update_interval_s = 0.5;
  double m_pos_delta_threshold_m = 0.2;
  double m_pos_update_max_interval_s = 1.0;

  DronePacketStats m_rx_stats;

  double m_total_distance_m = 0.0;
  bool m_has_prev_gt_pos = false;
  double m_prev_gt_x = 0.0;
  double m_prev_gt_y = 0.0;
  double m_prev_gt_z = 0.0;

  std::unordered_map<uint8_t, std::unordered_set<uint16_t>> m_relayed_ack_seqs;
  std::unordered_map<uint8_t, std::unordered_set<uint16_t>> m_relayed_pos_update_seqs;
  std::unordered_set<uint8_t> m_relayed_help_proxy;

  bool m_returning = false;
  bool m_return_triggered = false;
  double m_return_trigger_time_s = -1.0;
  double m_return_timeout_s = -1.0;
  bool m_return_flag_received = false;
  double m_return_start_s = -1.0;
  double m_return_complete_s = -1.0;
  bool m_return_trigger_was_flag = false;

  std::unordered_set<uint8_t> m_relayed_returning;

  static constexpr double RETURN_DELTA_S = 1.0;
  static constexpr uint8_t MAX_CHAIN_HOPS = 10;
  static constexpr float RETURN_K_ATT_SCALE = 0.3f;

  double m_first_ack_after_help_s = -1.0;
  Vector3D m_return_start_pos_gt{0.0, 0.0, 0.0};
  Vector3D m_return_complete_pos_gt{0.0, 0.0, 0.0};
  double   m_return_phase_distance_m = 0.0;
  uint32_t m_rearm_count = 0;
  Vector3D m_station_keeping_ref_pos_gt{0.0, 0.0, 0.0};
  double   m_station_keeping_max_drift_m = 0.0;

  bool   m_alive = true;
  double m_killed_at_s = -1.0;
};
