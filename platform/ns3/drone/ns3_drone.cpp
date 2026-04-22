#include "platform/ns3/drone/ns3_drone.h"

#include <cmath>

Ns3Drone::Ns3Drone(
  uint8_t id,
  ::ns3::Ptr<::ns3::Node> node,
  ControllerAlgorithm algorithm,
  float k_att,
  float k_rep,
  float d_safe,
  float v_max,
  float drone_weight_kg,
  double uwb_noise_std_dev_m
) :
  m_id(id),
  m_node(node),
  m_comm(std::make_unique<sim::UwbTransport>(node, id), id),
  m_controller(makeController(algorithm, id, k_att, k_rep, d_safe, v_max, drone_weight_kg))
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
  m_velocity_actuator = std::make_unique<Ns3VelocityActuator>(m_custom_mobility.get());

  m_comm.setReceiveHandler([this](const ::Packet& pkt) { dispatchPacket(pkt); });

  m_flood_manager = std::make_unique<FloodManager>(m_id, m_comm, [this]() { return isBaseReachable(); });
  m_neighbor_manager = std::make_unique<NeighborManager>(&m_comm);
  m_uwb_ranging_manager = std::make_unique<UwbRangingManager>(
    []() { return ::ns3::Simulator::Now().GetSeconds(); },
    299792458.0, uwb_noise_std_dev_m);
  m_uwb_position = std::make_unique<UwbPosition>(m_uwb_ranging_manager.get());

  // Seed the trilat estimate with spawn xy (and z=0).  Models an onboard
  // GPS/barometer snapshot at boot so drones that never achieve a 3-anchor
  // UWB fix don't report (0,0,0) forever (scenario coverage gaps like
  // far-corner drones can see only 1–2 anchors).
  //
  // We deliberately seed z=0 rather than spawn z.  When anchors are
  // effectively coplanar (our scenarios place them on short posts at
  // z ≈ 2–5 m), 2D trilat cannot update z — so whatever z the seed picks
  // is the z everyone uses forever.  Seeding spawn z creates an
  // open-loop integrator: the controller drives physical z toward
  // perceived-neighbor-z (including the base at z=0), but perceived self-z
  // never follows, so drones pile 50+ m into the ground chasing a target
  // they never seem to reach.  Seeding z=0 uniformly zeroes out all
  // z-deltas in the force model and leaves physical z at its spawn value.
  const ::ns3::Vector spawn = mobility->GetPosition();
  m_uwb_ranging_manager->seedEstimatedPosition(spawn.x, spawn.y, 0.0);
  m_uwb_position->retrieveCurrentPosition();

  m_dispatcher.setFloodManager(m_flood_manager.get());
  m_dispatcher.setNeighborManager(m_neighbor_manager.get());
  m_dispatcher.setUwbRangingManager(m_uwb_ranging_manager.get());
  m_dispatcher.setFallbackHandler([this](const ::Packet& pkt) { handleCorePacket(pkt); });

  m_last_ack_rx_s = ::ns3::Simulator::Now().GetSeconds();

  // Stagger periodic behavior to avoid all drones transmitting at the same instant.
  m_tick_phase_s = 0.01 * static_cast<double>(m_id);
}

void Ns3Drone::setBaseStation(uint8_t base_id) {
  m_base_id = base_id;
  m_has_base = true;

  m_last_ack_rx_s = ::ns3::Simulator::Now().GetSeconds();

  m_comm.registerPeer(base_id, 0);  // address unused with UWB transport
  if (m_flood_manager) {
    m_flood_manager->setBaseId(base_id);
  }
}

void Ns3Drone::start() {
  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_phase_s), ::ns3::MakeCallback(&Ns3Drone::onTick, this));
}

void Ns3Drone::setRepositionLogger(const std::shared_ptr<std::ofstream>& csv) {
  m_reposition_csv = csv;
}

void Ns3Drone::startMission() {
  if (!m_flood_manager || !m_velocity_actuator || !m_neighbor_manager || !m_uwb_position) {
    return;
  }

  m_controller->setMissionActive(true);

  m_mission_start_s = ::ns3::Simulator::Now().GetSeconds();
  m_last_mission_log_s = -1.0;

  std::cout << "[Mission] t=" << m_mission_start_s << "s drone=" << static_cast<int>(m_id)
            << " mission_active=1" << std::endl;
}

void Ns3Drone::stopMission() {
  m_controller->setMissionActive(false);
}

void Ns3Drone::kill() {
  if (!m_alive) return;
  m_alive = false;
  m_killed_at_s = ::ns3::Simulator::Now().GetSeconds();

  // Snapshot the exact kill position so currentGtPos() returns the frozen
  // spot (otherwise we'd report the last sample from up to m_tick_dt_s ago).
  if (auto mob = m_node ? m_node->GetObject<::ns3::ConstantPositionMobilityModel>() : nullptr) {
    const ::ns3::Vector p = mob->GetPosition();
    m_prev_gt_x = p.x;
    m_prev_gt_y = p.y;
    m_prev_gt_z = p.z;
  }

  if (m_velocity_actuator) {
    m_velocity_actuator->brake();
  }
  if (m_controller) {
    m_controller->setMissionActive(false);
    m_controller->setReturning(false);
    m_controller->setStationKeeping(false);
  }

  std::cout << "[KILL] t=" << m_killed_at_s << "s drone=" << static_cast<int>(m_id)
            << " pos=(" << m_prev_gt_x << "," << m_prev_gt_y << "," << m_prev_gt_z
            << ")" << std::endl;
}

void Ns3Drone::onTick() {
  // Dead drones: stop sampling, stop transmitting, and don't re-schedule —
  // the tick chain ends, saving event-queue work for the rest of the sim.
  if (!m_alive) {
    return;
  }
  const double now_s = ::ns3::Simulator::Now().GetSeconds();

  // Ground-truth distance accumulation.  We sample the mobility model (not the
  // trilaterated estimate) so UWB noise doesn't inflate the metric — the point
  // of the metric is to compare how much drones *actually* move across
  // formation-control variants.
  if (auto mob = m_node ? m_node->GetObject<::ns3::ConstantPositionMobilityModel>() : nullptr) {
    const ::ns3::Vector p = mob->GetPosition();
    if (m_has_prev_gt_pos) {
      const double dx = p.x - m_prev_gt_x;
      const double dy = p.y - m_prev_gt_y;
      const double dz = p.z - m_prev_gt_z;
      const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
      m_total_distance_m += step;
      if (m_returning) {
        m_return_phase_distance_m += step;
      }
    }
    m_prev_gt_x = p.x;
    m_prev_gt_y = p.y;
    m_prev_gt_z = p.z;
    m_has_prev_gt_pos = true;

    // Station-keeping drift: how far we've wandered from the position where
    // we first entered station-keeping.  Reset to 0 on entry (below).
    if (m_station_keeping) {
      const double dxr = p.x - m_station_keeping_ref_pos_gt.x;
      const double dyr = p.y - m_station_keeping_ref_pos_gt.y;
      const double dzr = p.z - m_station_keeping_ref_pos_gt.z;
      const double drift = std::sqrt(dxr * dxr + dyr * dyr + dzr * dzr);
      if (drift > m_station_keeping_max_drift_m) {
        m_station_keeping_max_drift_m = drift;
      }
    }
  }

  // Only emit HELP_PROXY if this drone is NOT already repositioning as a relay helper.
  // A mission-active drone may temporarily leave base coverage while transiting toward
  // the midpoint equilibrium — this is expected overshoot, not a loss of connectivity.
  if (m_waiting_ack && (now_s - m_last_ack_rx_s) > m_ack_timeout_s
      && !help_proxy_sent && !m_controller->isMissionActive()) {
    sendHelpProxy();
    m_waiting_ack = false;
  }

  // Platoon return: check if it's time to start returning (only the first
  // time — m_return_start_s latches on first trigger, so this block does not
  // re-fire after station-keeping.  Legitimate re-loss is handled by the
  // safety re-arm block below, which does not touch m_return_start_s.)
  if (help_proxy_sent && m_return_triggered && !m_returning && m_return_start_s < 0.0) {
    const bool timeout_expired = (now_s - m_return_trigger_time_s) >= m_return_timeout_s;

    // Start returning if:
    // (a) timeout expired (safety net — covers packet loss and tail nodes), OR
    // (b) we received a RETURNING flag from a downstream (higher-hop) drone
    if (timeout_expired || m_return_flag_received) {
      m_returning = true;
      m_return_start_s = now_s;
      m_return_trigger_was_flag = !timeout_expired;
      m_controller->setReturning(true);

      // Snapshot the ground-truth position where the return phase starts and
      // reset path accumulator — the return-quality metric compares the path
      // length travelled during return vs. the straight-line distance.
      m_return_start_pos_gt = Vector3D(m_prev_gt_x, m_prev_gt_y, m_prev_gt_z);
      m_return_phase_distance_m = 0.0;

      // Broadcast our own RETURNING flag so upstream drones can start
      const uint8_t my_hops = m_flood_manager ? m_flood_manager->getHopsFromBase() : UINT8_MAX;
      ReturningMsg ret;
      ret.drone_id = m_id;
      ret.hop_count = my_hops;

      ::Packet out;
      out.type = ::PacketType::CORE;
      out.src = m_id;
      out.dst = BROADCAST_ID;
      out.payload.resize(sizeof(ret));
      std::memcpy(out.payload.data(), &ret, sizeof(ret));
      m_comm.send(out);

      std::cout << "[RETURNING] t=" << now_s << "s drone=" << static_cast<int>(m_id)
                << " hops=" << static_cast<int>(my_hops)
                << " trigger=" << (timeout_expired ? "timeout" : "flag") << std::endl;
    }
  }

  // Exit returning mode: back in direct base coverage.  Use direct-ACK
  // reception (tracked independently of m_last_ack_rx_s) rather than the
  // flood-based hops value, which lags by a full flood round.  On entry
  // we brake explicitly and enter station-keeping so the controller holds
  // position instead of coasting.
  if (m_returning && hasDirectBaseCoverage()) {
    m_returning = false;
    m_return_complete_s = now_s;
    m_station_keeping = true;
    m_controller->setReturning(false);
    m_controller->setStationKeeping(true);
    if (m_velocity_actuator) {
      m_velocity_actuator->brake();
    }

    // Snapshot ground-truth position where the return phase ended.  This is
    // both the end-point for return-path-efficiency/boundary metrics and the
    // reference for station-keeping drift.
    m_return_complete_pos_gt = Vector3D(m_prev_gt_x, m_prev_gt_y, m_prev_gt_z);
    m_station_keeping_ref_pos_gt = m_return_complete_pos_gt;
    m_station_keeping_max_drift_m = 0.0;

    std::cout << "[RETURN_COMPLETE] t=" << now_s << "s drone=" << static_cast<int>(m_id) << std::endl;
  }

  // Safety re-arm: if we've been station-keeping but fell out of direct
  // coverage for longer than 2 * DIRECT_ACK_TIMEOUT_S, treat it as a
  // re-loss and go back to return mode.
  if (m_station_keeping && !hasDirectBaseCoverage()) {
    const double since = (m_last_direct_ack_rx_s < 0.0)
      ? 1e9
      : (now_s - m_last_direct_ack_rx_s);
    if (since > 2.0 * DIRECT_ACK_TIMEOUT_S) {
      m_station_keeping = false;
      m_returning = true;
      m_rearm_count += 1;
      m_controller->setStationKeeping(false);
      m_controller->setReturning(true);
      std::cout << "[RETURN_REARM] t=" << now_s << "s drone=" << static_cast<int>(m_id)
                << " lost direct coverage after " << since << "s" << std::endl;
    }
  }

  // Post-mission debug: log how drones reposition
  if (m_mission_start_s >= 0.0) {
    if (m_last_mission_log_s < 0.0 || (now_s - m_last_mission_log_s) >= m_mission_log_dt_s) {
      if (m_uwb_position) {
        m_uwb_position->retrieveCurrentPosition();
        const auto coords = m_uwb_position->getCoordinates();
        const uint8_t hops = m_flood_manager ? m_flood_manager->getHopsFromBase() : UINT8_MAX;
        const size_t n_neighbors = m_neighbor_manager ? m_neighbor_manager->getNeighbors().size() : 0;
        std::cout << "[Reposition] t=" << now_s << "delta_t" << (now_s - m_last_mission_log_s) << "s drone=" << static_cast<int>(m_id)
                  << " hops=" << static_cast<int>(hops)
                  << " neighbors=" << n_neighbors
                  << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
                  << "," << (coords.size() > 1 ? coords[1] : 0.0)
                  << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")";

        if (m_last_help_proxy_rx_s >= 0.0) {
          std::cout << " after_HELP_PROXY_RX(t=" << m_last_help_proxy_rx_s << "s)";
        }
        if (m_last_help_proxy_tx_s >= 0.0) {
          std::cout << " after_HELP_PROXY_TX(t=" << m_last_help_proxy_tx_s << "s)";
        }
        std::cout << std::endl;

        if (m_reposition_csv && m_reposition_csv->good()) {
          (*m_reposition_csv) << now_s << ","
                              << static_cast<int>(m_id) << ","
                              << static_cast<int>(hops) << ","
                              << n_neighbors << ","
                              << (coords.size() > 0 ? coords[0] : 0.0) << ","
                              << (coords.size() > 1 ? coords[1] : 0.0) << ","
                              << (coords.size() > 2 ? coords[2] : 0.0)
                              << std::endl;
        }
      }
      m_last_mission_log_s = now_s;
    }
  }

  // Drive motion in lockstep with simulation time.
  // - If mission is active: one potential-field iteration per tick.
  // - If mission is off: apply a default "idle" velocity so drones move and can leave coverage.
  if (m_flood_manager && m_velocity_actuator && m_neighbor_manager && m_uwb_position) {
    m_uwb_position->retrieveCurrentPosition();
    m_controller->step(
      m_flood_manager.get(),
      m_velocity_actuator.get(),
      m_neighbor_manager.get(),
      m_uwb_position.get()
    );
  }

  // Always send periodic updates so reachability (ACK-based) is meaningful.
  // HELP_PROXY is only emitted when we are not in mission.
  sendPositionUpdate();

  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_dt_s), ::ns3::MakeCallback(&Ns3Drone::onTick, this));
}

void Ns3Drone::dispatchPacket(const ::Packet& pkt) {
  if (!m_alive) {
    return;  // Dead drones drop all incoming packets — no state changes, no relays.
  }
  if (pkt.payload.empty()) {
    return;
  }

  if (pkt.dst != m_id && pkt.dst != BROADCAST_ID) {
    return;
  }

  // Count by packet category (CORE sub-types are counted inside handleCorePacket).
  switch (pkt.type) {
    case ::PacketType::FLOOD:      ++m_rx_stats.flood;      break;
    case ::PacketType::NEIGHBOR:   ++m_rx_stats.neighbor;   break;
    case ::PacketType::UWB_BEACON: ++m_rx_stats.uwb_beacon; break;
    default: break;
  }

  m_dispatcher.handlePacket(pkt);
}

void Ns3Drone::handleCorePacket(const ::Packet& pkt) {
  if (pkt.payload.size() < 1) {
    return;
  }

  const auto type = static_cast<SimMsgType>(pkt.payload[0]);
  switch (type) {
    case SimMsgType::POS_ACK: {
      ++m_rx_stats.pos_ack;
      if (pkt.payload.size() < sizeof(PositionAckMsg)) {
        return;
      }
      PositionAckMsg ack;
      std::memcpy(&ack, pkt.payload.data(), sizeof(ack));

      if (ack.base_id != m_base_id ) {
        return;
      }
      if (ack.drone_id != m_id) {
        // Relay ACKs for other drones to support multi-hop chains (scenario 2).
        // Dedup by (drone_id, seq) so each unique ACK is relayed at most once,
        // preventing broadcast loops without breaking deeper relay hops.
        auto& relayed = m_relayed_ack_seqs[ack.drone_id];
        if (relayed.count(ack.seq)) {
          return;  // already relayed this ACK
        }
        relayed.insert(ack.seq);
        ::Packet relay_pkt;
        relay_pkt.type = ::PacketType::CORE;
        // Set src to this relayer so downstream drones can distinguish
        // direct ACKs (pkt.src == ack.base_id) from relayed ones.
        relay_pkt.src = m_id;
        relay_pkt.dst = BROADCAST_ID;
        relay_pkt.payload.resize(sizeof(ack));
        std::memcpy(relay_pkt.payload.data(), &ack, sizeof(ack));
        m_comm.send(relay_pkt);
        return;
      }

      // Distinguish direct ACKs (arrived straight from base) from relayed
      // ones (the relayer sets pkt.src to its own id above).  Direct ACKs
      // refresh m_last_direct_ack_rx_s unconditionally — that timestamp is
      // what hasDirectBaseCoverage() / the station-keeping trigger consult.
      const bool is_direct = (pkt.src == ack.base_id);
      if (is_direct) {
        m_last_direct_ack_rx_s = ::ns3::Simulator::Now().GetSeconds();
      }

      // Healing-latency metric: first ACK addressed to us after our HELP_PROXY
      // marks the moment the relay chain is operational end-to-end.
      if (help_proxy_sent && m_first_ack_after_help_s < 0.0) {
        m_first_ack_after_help_s = ::ns3::Simulator::Now().GetSeconds();
      }

      // Don't update m_last_ack_rx_s if we've sent HELP_PROXY - we've lost direct
      // connectivity and relayed ACKs shouldn't make us think we're connected again.
      // This is critical for the flood hop-count calculation to remain accurate.
      if (!help_proxy_sent) {
        m_last_ack_rx_s = ::ns3::Simulator::Now().GetSeconds();
      } else {
        // Log when the lost drone receives a relayed ACK
        std::cout << "[RELAYED_ACK_RX] t=" << ::ns3::Simulator::Now().GetSeconds()
                  << "s drone=" << static_cast<int>(m_id)
                  << " seq=" << ack.seq << std::endl;

        // First relayed ACK with a valid hop count confirms the relay chain
        // is operational.  Don't arm if hops are still unknown (UINT8_MAX) —
        // the flood table hasn't converged yet and we'd compute timeout=0.
        const uint8_t hops = m_flood_manager ? m_flood_manager->getHopsFromBase() : UINT8_MAX;
        if (!m_return_triggered && hops != UINT8_MAX) {
          m_return_triggered = true;
          m_return_trigger_time_s = ::ns3::Simulator::Now().GetSeconds();
          m_return_timeout_s = static_cast<double>(MAX_CHAIN_HOPS - std::min(hops, MAX_CHAIN_HOPS)) * RETURN_DELTA_S;

          std::cout << "[RETURN_ARMED] t=" << m_return_trigger_time_s
                    << "s drone=" << static_cast<int>(m_id)
                    << " hops=" << static_cast<int>(hops)
                    << " timeout=" << m_return_timeout_s << "s" << std::endl;
        }
      }
      m_last_acked_seq = ack.seq;
      m_waiting_ack = false;

      // Treat the base station as a regular neighbor entry — but ONLY
      // on direct ACKs.  Synthesizing this from relayed ACKs plants a
      // (0,0,0)/hops=0 attractor in the neighbor table of lost drones,
      // which then pulls them all the way to the origin during return
      // mode instead of letting them stop at the coverage boundary.
      // Payload format matches NeighborManager broadcasts:
      // [id][hops][flags][double coords...].
      if (is_direct && m_neighbor_manager) {
        ::Packet base_as_neighbor;
        base_as_neighbor.type = ::PacketType::NEIGHBOR;
        base_as_neighbor.src = ack.base_id;
        base_as_neighbor.dst = m_id;

        base_as_neighbor.payload.resize(3 + 3 * sizeof(double));
        base_as_neighbor.payload[0] = ack.base_id;
        base_as_neighbor.payload[1] = ack.base_hops_to_base_station;
        base_as_neighbor.payload[2] = 0;  // flags: base is never "returning"

        const double base_coords[3] = {ack.x, ack.y, ack.z};
        std::memcpy(base_as_neighbor.payload.data() + 3, base_coords, sizeof(base_coords));

        m_neighbor_manager->onPacketReceived(base_as_neighbor);
      }
      return;
    }

    case SimMsgType::HELP_PROXY: {
      ++m_rx_stats.help_proxy;
      if (pkt.payload.size() < sizeof(HelpProxyMsg)) {
        return;
      }
      HelpProxyMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));

      // Only react to HELP_PROXY requests that target our base station.
      if (msg.base_id != m_base_id) {
        return;
      }

      if (msg.requester_id == m_id) {
        return; // ignore our own HELP_PROXY
      }

      m_last_help_proxy_rx_s = ::ns3::Simulator::Now().GetSeconds();
      if (m_uwb_position) {
        m_uwb_position->retrieveCurrentPosition();
        const auto coords = m_uwb_position->getCoordinates();
        std::cout << "[HELP_PROXY RX] t=" << m_last_help_proxy_rx_s << "s drone=" << static_cast<int>(m_id)
                  << " requester=" << static_cast<int>(msg.requester_id)
                  << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
                  << "," << (coords.size() > 1 ? coords[1] : 0.0)
                  << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")" << std::endl;
      }

      if (!help_proxy_sent && isBaseReachable()) {
        // Only in-coverage drones (base reachable) enter mission mode to reposition.
        // Drones that have already lost base connectivity must not start mission —
        // they are not in a position to serve as relay nodes.
        startMission();
      } else if (!isBaseReachable()) {
        // This drone is itself lost (or about to time out). Relay the HELP_PROXY upstream
        // so in-coverage drones further up the chain discover all nodes (scenario 2).
        // Dedup by requester_id: each lost drone relays each unique requester's message once.
        if (!m_relayed_help_proxy.count(msg.requester_id)) {
          m_relayed_help_proxy.insert(msg.requester_id);
          ::Packet relay;
          relay.type = ::PacketType::CORE;
          relay.src = m_id;
          relay.dst = BROADCAST_ID;
          relay.payload.resize(sizeof(msg));
          std::memcpy(relay.payload.data(), &msg, sizeof(msg));
          m_comm.send(relay);
        }
      }
      return;
    }

    case SimMsgType::RETURNING: {
      if (pkt.payload.size() < sizeof(ReturningMsg)) return;
      ReturningMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));

      if (msg.drone_id == m_id) return;  // ignore our own

      // Only act on flags from the immediate next-hop neighbor (hop x+1),
      // not from arbitrary higher-hop drones further down the chain.
      const uint8_t my_hops = m_flood_manager ? m_flood_manager->getHopsFromBase() : UINT8_MAX;
      if (msg.hop_count == my_hops + 1 && m_return_triggered && !m_returning && !m_return_flag_received) {
        m_return_flag_received = true;
        std::cout << "[RETURN_FLAG_RX] t=" << ::ns3::Simulator::Now().GetSeconds()
                  << "s drone=" << static_cast<int>(m_id)
                  << " from=" << static_cast<int>(msg.drone_id)
                  << " (hop " << static_cast<int>(msg.hop_count) << ")" << std::endl;
      }

      // Relay the flag upstream (dedup by drone_id)
      if (!m_relayed_returning.count(msg.drone_id)) {
        m_relayed_returning.insert(msg.drone_id);
        ::Packet relay;
        relay.type = ::PacketType::CORE;
        relay.src = m_id;
        relay.dst = BROADCAST_ID;
        relay.payload.resize(sizeof(msg));
        std::memcpy(relay.payload.data(), &msg, sizeof(msg));
        m_comm.send(relay);
      }
      return;
    }

    case SimMsgType::POS_UPDATE:{
      ++m_rx_stats.pos_update;
      if (pkt.payload.size() < sizeof(PositionUpdateMsg)) {
        return;
      }
      PositionUpdateMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));

      // Ignore updates not for our base station.
      if (msg.base_id != m_base_id) {
        return;
      }

      // Don't relay our own position updates.
      if (msg.drone_id == m_id) {
        return;
      }

      // Only relay broadcasts (from lost drones) to base station.
      // If the packet was already unicast to base, don't relay.
      if (pkt.dst != BROADCAST_ID) {
        return;
      }

      // Dedup by (drone_id, seq): relay each unique POS_UPDATE at most once.
      // Without this, multiple lost drones in a mesh relay each other's updates
      // indefinitely — none of them are the originator, so the originator-id
      // check above never fires, creating an infinite broadcast storm.
      auto& relayed = m_relayed_pos_update_seqs[msg.drone_id];
      if (relayed.count(msg.seq)) {
        return;  // already relayed this update
      }
      relayed.insert(msg.seq);

      // Relay to base station.
      ::Packet relay_pkt;
      relay_pkt.type = ::PacketType::CORE;
      relay_pkt.src = m_id;  // use our id as sender
      relay_pkt.dst = help_proxy_sent ? BROADCAST_ID : m_base_id;
      relay_pkt.payload.resize(sizeof(msg));
      std::memcpy(relay_pkt.payload.data(), &msg, sizeof(msg));

      m_comm.send(relay_pkt);
      return;
    }

    default:
      return;
  }
}

void Ns3Drone::sendPositionUpdate() {
  if (!m_uwb_position || !m_has_base) {
    return;
  }

  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  if ((now_s - m_last_pos_send_s) < m_pos_update_interval_s) {
    return;
  }

  m_uwb_position->retrieveCurrentPosition();
  const auto coords = m_uwb_position->getCoordinates();

  // Delta gate: once the minimum 500 ms has elapsed, skip this send if the
  // drone hasn't moved enough AND we're still within the max-interval window.
  // The max-interval fallback guarantees a heartbeat even when the drone is
  // perfectly stationary, so the base's reachability logic keeps working.
  const double cur_x = coords.size() > 0 ? coords[0] : 0.0;
  const double cur_y = coords.size() > 1 ? coords[1] : 0.0;
  const double cur_z = coords.size() > 2 ? coords[2] : 0.0;
  if (m_has_last_sent_pos
      && (now_s - m_last_pos_send_s) < m_pos_update_max_interval_s) {
    const double dx = cur_x - m_last_sent_x;
    const double dy = cur_y - m_last_sent_y;
    const double dz = cur_z - m_last_sent_z;
    const double dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq < (m_pos_delta_threshold_m * m_pos_delta_threshold_m)) {
      return;
    }
  }

  // If help proxy was sent, send position updates via broadcast to inform helpers.
  // Otherwise, unicast to base station.

  PositionUpdateMsg pos;
  pos.drone_id = m_id;
  pos.base_id = m_base_id;
  pos.seq = ++m_pos_seq;
  pos.x = coords.size() > 0 ? static_cast<float>(coords[0]) : 0.0f;
  pos.y = coords.size() > 1 ? static_cast<float>(coords[1]) : 0.0f;
  pos.z = coords.size() > 2 ? static_cast<float>(coords[2]) : 0.0f;

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = help_proxy_sent ? BROADCAST_ID : m_base_id;
  out.payload.resize(sizeof(pos));
  std::memcpy(out.payload.data(), &pos, sizeof(pos)); 

  m_comm.send(out);
  m_last_pos_send_s = now_s;
  m_last_sent_x = cur_x;
  m_last_sent_y = cur_y;
  m_last_sent_z = cur_z;
  m_has_last_sent_pos = true;
  m_waiting_ack = true;
}

void Ns3Drone::sendHelpProxy() {
  m_last_help_proxy_tx_s = ::ns3::Simulator::Now().GetSeconds();

  if (m_uwb_position) {
    m_uwb_position->retrieveCurrentPosition();
    const auto coords = m_uwb_position->getCoordinates();
    std::cout << "[HELP_PROXY TX] t=" << m_last_help_proxy_tx_s << "s drone=" << static_cast<int>(m_id)
              << " reason=ACK_TIMEOUT"
              << " last_ack=" << m_last_ack_rx_s << "s"
              << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
              << "," << (coords.size() > 1 ? coords[1] : 0.0)
              << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")" << std::endl;
  } else {
    std::cout << "[HELP_PROXY TX] t=" << m_last_help_proxy_tx_s << "s drone=" << static_cast<int>(m_id)
              << " reason=ACK_TIMEOUT" << std::endl;
  }

  HelpProxyMsg help;
  help.requester_id = m_id;
  help.base_id = m_base_id;

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = BROADCAST_ID;
  out.payload.resize(sizeof(help));
  std::memcpy(out.payload.data(), &help, sizeof(help));

  m_comm.send(out);

  stopMission();
  help_proxy_sent = true;
}

bool Ns3Drone::isBaseReachable() const {
  if (!m_has_base) {
    return false;
  }

  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  return (now_s - m_last_ack_rx_s) <= m_ack_timeout_s;
}

bool Ns3Drone::hasDirectBaseCoverage() const {
  if (m_last_direct_ack_rx_s < 0.0) {
    return false;
  }
  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  return (now_s - m_last_direct_ack_rx_s) <= DIRECT_ACK_TIMEOUT_S;
}