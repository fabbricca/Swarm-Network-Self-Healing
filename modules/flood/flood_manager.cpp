#include "modules/flood/flood_manager.h"

FloodManager::FloodManager(
    uint8_t self,
    CommunicationManagerInterface& cm,
    std::function<bool()> base_reachable_fn
) :
    self_id(self),
    communication_manager(cm),
    is_base_reachable(std::move(base_reachable_fn))
{ }

void FloodManager::onPacketReceived(const ::Packet& pkt) {
    if (pkt.payload.empty()) {
        return;
    }

    auto type = static_cast<FloodMsgType>(pkt.payload[0]);

    switch(type) {
        case FloodMsgType::START: {
            FloodStartMsg msg;
            if (decodeStart(pkt, msg)) {
                handleStart(msg);
            }
            break;
        }
        case FloodMsgType::DISCOVERY: {
            FloodDiscoveryMsg msg;
            if (decodeDiscovery(pkt, msg)) {
                handleDiscovery(msg);
            }
            break;
        }
        case FloodMsgType::REPORT: {
            FloodReportMsg msg;
            if (decodeReport(pkt, msg)) {
                handleReport(msg);
            }
            break;
        }
        default:
            // unrecognized message type
            return;
    }
}

void FloodManager::setBaseId(uint8_t base) {
    base_id = base;
}

uint8_t FloodManager::getHopsFromBase() const {
    if (is_base_reachable && is_base_reachable()) {
        return 1;
    }

    // higher flood_id means more recent flood
    uint16_t latest_flood_id = 0;
    for (auto [flood_id, hops_to_base] : best_hop_to_base) {
        latest_flood_id = (latest_flood_id < flood_id) ? flood_id : latest_flood_id;
    }
    auto it = best_hop_to_base.find(latest_flood_id);
    if (it == best_hop_to_base.end()) {
        // no floods seen yet
        return UINT8_MAX;
    }

    // If the base is not reachable now, we must not report a direct hop (1)
    // based on stale flood state from when we *were* directly connected.
    if (it->second == 1) {
        return UINT8_MAX;
    }

    return static_cast<uint8_t>(it->second);
}

void FloodManager::startFlood(uint16_t flood_id) {
    // Initiator seeds the flood.
    FloodDiscoveryMsg msg;
    msg.base_id = base_id;
    msg.flood_id = flood_id;
    msg.initiator_id = self_id;
    // The initiator IS 1 hop from base.  Broadcast this actual hop count so
    // that non-base-reachable recipients compute 1+1=2 (correct) instead of
    // 0+1=1 which collides with the stale-detection filter in getHopsFromBase().
    msg.hop_to_base = 1;

    seen_floods.insert(flood_id);
    best_hop_to_base[flood_id] = 1;

    ::Packet pkt;
    pkt.type = ::PacketType::FLOOD;
    pkt.src = self_id;
    pkt.dst = BROADCAST_ID;
    pkt.payload.resize(sizeof(msg));
    std::memcpy(pkt.payload.data(), &msg, sizeof(msg));

    communication_manager.send(pkt);
}

void FloodManager::handleStart(const FloodStartMsg& msg) {
    // Base requests this node to act as initiator.
    // Avoid restarting the same flood multiple times.
    if (seen_floods.count(msg.flood_id)) {
        return;
    }
    startFlood(msg.flood_id);
}

void FloodManager::handleDiscovery(const FloodDiscoveryMsg& msg) {
    // Multi-base partition: ignore discovery from a base this node isn't
    // attached to.  Prevents cross-swarm hop-count contamination.
    if (msg.base_id != base_id) {
        return;
    }

    const uint16_t flood_id = msg.flood_id;
    const uint8_t initiator_id = msg.initiator_id;

    // Compute candidate hop to initiator.
    const bool base_reachable = (is_base_reachable && is_base_reachable());
    const uint8_t candidate_hop = base_reachable
        ? static_cast<uint8_t>(1)
        : static_cast<uint8_t>(msg.hop_to_base + 1);

    bool improved = false;
    auto it = best_hop_to_base.find(flood_id);
    if (it == best_hop_to_base.end()) {
        improved = true;
        best_hop_to_base[flood_id] = candidate_hop;
        seen_floods.insert(flood_id);
    } else if (candidate_hop < it->second) {
        improved = true;
        it->second = candidate_hop;
    }

    if (!improved) {
        return;
    }

    // Mark our own report as seen so we don't forward an echoed copy later.
    best_report_seen[flood_id][self_id] = candidate_hop;
    ::Packet report_pkt = createReportMsg(flood_id, initiator_id, candidate_hop);
    communication_manager.send(report_pkt);

    // Rebroadcast discovery with incremented hop.
    ::Packet flood_pkt = createDiscoveryMsg(flood_id, initiator_id, candidate_hop);
    communication_manager.send(flood_pkt);
}

void FloodManager::handleReport(const FloodReportMsg& msg) {
    // Multi-base partition: discard reports for other bases.
    if (msg.base_id != base_id) {
        return;
    }

    // Ignore reports for floods we never joined (limits propagation scope).
    if (!seen_floods.count(msg.flood_id)) {
        return;
    }

    // Forward each reporter's best-known report at most once per improvement.
    bool improved = false;
    auto& seen = best_report_seen[msg.flood_id];
    auto it_seen = seen.find(msg.reporter_id);
    if (it_seen == seen.end() || msg.hop_to_base < it_seen->second) {
        improved = true;
        seen[msg.reporter_id] = msg.hop_to_base;
    }

    if (!improved) {
        return;
    }

    // Non-initiators rebroadcast reports so they can reach the initiator over multiple hops.
    ::Packet report_pkt = createReportMsg(msg.flood_id, msg.initiator_id, msg.hop_to_base);
    communication_manager.send(report_pkt);
}

::Packet FloodManager::createReportMsg(uint16_t flood_id, uint8_t initiator_id, uint8_t candidate_hop) {
    // Create a report with the best hop we currently know.
    FloodReportMsg report;
    report.base_id = base_id;
    report.flood_id = flood_id;
    report.initiator_id = initiator_id;
    report.reporter_id = self_id;
    report.hop_to_base = candidate_hop;

    ::Packet report_pkt;
    report_pkt.type = ::PacketType::FLOOD;
    report_pkt.src = self_id;
    report_pkt.dst = BROADCAST_ID;
    report_pkt.payload.resize(sizeof(report));
    std::memcpy(report_pkt.payload.data(), &report, sizeof(report));
    return report_pkt;
}

::Packet FloodManager::createDiscoveryMsg(uint16_t flood_id, uint8_t initiator_id, uint8_t hop_to_base) {
    // Create a discovery message to rebroadcast.
    FloodDiscoveryMsg msg;
    msg.base_id = base_id;
    msg.flood_id = flood_id;
    msg.initiator_id = initiator_id;
    msg.hop_to_base = hop_to_base;

    ::Packet pkt;
    pkt.type = ::PacketType::FLOOD;
    pkt.src = self_id;
    pkt.dst = BROADCAST_ID;
    pkt.payload.resize(sizeof(msg));
    std::memcpy(pkt.payload.data(), &msg, sizeof(msg));
    return pkt;
}

bool FloodManager::decodeStart(const ::Packet& pkt, FloodStartMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodStartMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodStartMsg));
    return msg.type == FloodMsgType::START;
}

bool FloodManager::decodeDiscovery(const ::Packet& pkt, FloodDiscoveryMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodDiscoveryMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodDiscoveryMsg));
    return msg.type == FloodMsgType::DISCOVERY;
}

bool FloodManager::decodeReport(const ::Packet& pkt, FloodReportMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodReportMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodReportMsg));
    return msg.type == FloodMsgType::REPORT;
}
