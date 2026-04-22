#pragma once

#include <cstdint>
#include <memory>
#include <cstring>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>

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

// NS-3 bound drone node logic.
// - While not in mission: periodically unicast PositionUpdateMsg to base and wait for PositionAckMsg.
// - If ACK is missing for too long: broadcast HelpProxyMsg.
// - When mission starts: Controller drives motion and NeighborManager broadcasts to neighbors.
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

  void setBaseStation(uint8_t base_id);

  void startMission();
  void stopMission();

  void start();

  void setRepositionLogger(const std::shared_ptr<std::ofstream>& csv);

  const DronePacketStats& rxStats() const { return m_rx_stats; }

  double totalDistanceTraveled() const { return m_total_distance_m; }

  uint8_t hopsFromBase() const {
    return m_flood_manager ? m_flood_manager->getHopsFromBase() : 0xFF;
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

  // Mid-sim scheduled failure: freezes the drone in place, stops all TX/RX,
  // and bypasses controller state.  One-shot — no revive path.
  void   kill();
  bool   isAlive() const { return m_alive; }
  double killedAtS() const { return m_killed_at_s; }

 private:
  void onTick();
  void dispatchPacket(const ::Packet& pkt);
  void handleCorePacket(const ::Packet& pkt);

  void sendPositionUpdate();
  void sendHelpProxy();

  bool isBaseReachable() const;
  bool hasDirectBaseCoverage() const;

  uint8_t m_id;
  ::ns3::Ptr<::ns3::Node> m_node;

  std::unique_ptr<CustomMobility> m_custom_mobility;
  std::unique_ptr<Ns3VelocityActuator> m_velocity_actuator;

  uint8_t m_base_id = 0;
  bool m_has_base = false;

  CommunicationManager m_comm;

  std::unique_ptr<FloodManager> m_flood_manager;
  std::unique_ptr<NeighborManager> m_neighbor_manager;
  std::unique_ptr<UwbRangingManager> m_uwb_ranging_manager;
  std::unique_ptr<UwbPosition> m_uwb_position;
  DispatchManager m_dispatcher;

  std::unique_ptr<ControllerInterface> m_controller;

  bool help_proxy_sent = false;

  // Debug logging for mission transitions / post-HELP_PROXY repositioning.
  double m_last_help_proxy_tx_s = -1.0;
  double m_last_help_proxy_rx_s = -1.0;

  double m_mission_start_s = -1.0;
  double m_last_mission_log_s = -1.0;
  double m_mission_log_dt_s = 0.5;

  double m_last_idle_log_s = -1.0;
  double m_idle_log_dt_s = 2.0;

  std::shared_ptr<std::ofstream> m_reposition_csv;

  // Heartbeat/ack tracking (reachability is based on receiving ACKs)
  double m_tick_dt_s = 0.05;
  double m_tick_phase_s = 0.0;

  double m_ack_timeout_s = 1.5;
  double m_last_ack_rx_s = 0.0;
  bool m_waiting_ack = false;

  // Direct-coverage tracking: refreshed only by POS_ACKs received directly
  // from the base (pkt.src == ack.base_id).  This bypasses the help-proxy
  // freeze on m_last_ack_rx_s so we can tell when a returning drone has
  // re-entered the base's radio cell.
  double m_last_direct_ack_rx_s = -1.0;
  static constexpr double DIRECT_ACK_TIMEOUT_S = 1.5;

  // Station-keeping: active once we complete the return and sit at the
  // coverage boundary.  Controller short-circuits to brake().
  bool m_station_keeping = false;

  uint16_t m_pos_seq = 0;
  uint16_t m_last_acked_seq = 0;
  double m_last_pos_send_s = 0.0;
  // POS_UPDATE is decoupled from the 20 Hz physics tick: the base only needs
  // fresh-enough reachability evidence, and broadcasting 20×/s per drone was
  // the dominant CORE-traffic source.
  double m_pos_update_interval_s = 0.5;

  // Delta-based gate on POS_UPDATE: once the 500 ms minimum has passed, skip
  // the send if the drone hasn't moved meaningfully since the previous send.
  // The max-interval fallback must stay below m_ack_timeout_s (1.5 s) — a
  // stationary drone still needs regular heartbeats so the base doesn't
  // declare it lost and so it doesn't self-trigger HELP_PROXY.
  double m_last_sent_x = 0.0;
  double m_last_sent_y = 0.0;
  double m_last_sent_z = 0.0;
  bool m_has_last_sent_pos = false;
  double m_pos_delta_threshold_m = 0.2;
  double m_pos_update_max_interval_s = 1.0;

  DronePacketStats m_rx_stats;

  // Ground-truth distance accumulation (sampled each physics tick from the
  // mobility model, not the trilaterated estimate, so noise doesn't inflate
  // the metric).
  double m_total_distance_m = 0.0;
  bool m_has_prev_gt_pos = false;
  double m_prev_gt_x = 0.0;
  double m_prev_gt_y = 0.0;
  double m_prev_gt_z = 0.0;

  // Multi-hop ACK relay: track which (drone_id, seq) pairs we've already relayed
  // to prevent broadcast loops while still allowing chained relay beyond 1 hop.
  std::unordered_map<uint8_t, std::unordered_set<uint16_t>> m_relayed_ack_seqs;

  // Multi-hop POS_UPDATE relay: same dedup pattern as ACK relay — prevents the
  // broadcast storm that occurs when multiple lost drones form a mesh and each
  // relay the same POS_UPDATE packet indefinitely.
  std::unordered_map<uint8_t, std::unordered_set<uint16_t>> m_relayed_pos_update_seqs;

  // HELP_PROXY relay: lost drones relay other lost drones' HELP_PROXY upstream
  // so in-coverage drones discover all nodes in the chain.
  std::unordered_set<uint8_t> m_relayed_help_proxy;

  // ── Platoon-based returning mode ──
  bool m_returning = false;              // true once this lost drone starts moving back
  bool m_return_triggered = false;       // true once first relayed ACK received (chain exists)
  double m_return_trigger_time_s = -1.0; // when the first relayed ACK arrived
  double m_return_timeout_s = -1.0;      // computed: (MAX_CHAIN_HOPS - my_hops) * RETURN_DELTA_S
  bool m_return_flag_received = false;   // true if a downstream RETURNING flag was received
  double m_return_start_s = -1.0;        // when this drone actually started moving back
  double m_return_complete_s = -1.0;     // when this drone reached hop 1
  bool m_return_trigger_was_flag = false; // true if return triggered by flag, false if by timeout

  // Dedup: only relay each drone's RETURNING flag once
  std::unordered_set<uint8_t> m_relayed_returning;

  // Tuning constants
  static constexpr double RETURN_DELTA_S = 1.0;     // timeout gap per hop level
  static constexpr uint8_t MAX_CHAIN_HOPS = 10;     // upper bound for timeout calc
  static constexpr float RETURN_K_ATT_SCALE = 0.3f; // reduced attraction gain for returning

  // ── End-of-sim metrics telemetry ──
  // Healing-latency: first ACK addressed to us received after help_proxy_sent.
  double m_first_ack_after_help_s = -1.0;

  // Return-quality: ground-truth snapshots + path-length accumulation.
  Vector3D m_return_start_pos_gt{0.0, 0.0, 0.0};
  Vector3D m_return_complete_pos_gt{0.0, 0.0, 0.0};
  double   m_return_phase_distance_m = 0.0;
  uint32_t m_rearm_count = 0;

  // Post-return stability: drift from the position captured at station-keeping entry.
  Vector3D m_station_keeping_ref_pos_gt{0.0, 0.0, 0.0};
  double   m_station_keeping_max_drift_m = 0.0;

  // Mid-sim failure.
  bool   m_alive = true;
  double m_killed_at_s = -1.0;
};
