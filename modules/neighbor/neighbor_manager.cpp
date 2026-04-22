#include "modules/neighbor/neighbor_manager.h"

NeighborManager::NeighborManager(
    CommunicationManagerInterface* communication_manager
) : 
    m_communication_manager(communication_manager) 
{ }

void NeighborManager::onPacketReceived(const ::Packet& pkt) {
    if (pkt.type != ::PacketType::NEIGHBOR) {
        return;
    }
    if (pkt.payload.size() < 3) {
        return;
    }

    // Payload format: [neighbor_id][hops][flags][double coords...]
    const uint8_t neighbor_id = pkt.payload[0];
    if (neighbor_id != pkt.src) {
        // Basic sanity check: outer header src should match payload id.
        return;
    }

    const uint8_t hops = pkt.payload[1];
    const uint8_t flags = pkt.payload[2];
    const bool returning = (flags & NeighborInfo::FLAG_RETURNING) != 0;
    const size_t coord_bytes = pkt.payload.size() - 3;
    if (coord_bytes % sizeof(double) != 0) {
        return;
    }

    std::vector<double> coords;
    coords.resize(coord_bytes / sizeof(double));
    if (coord_bytes > 0) {
        std::memcpy(coords.data(), pkt.payload.data() + 3, coord_bytes);
    }

    auto& entry = m_neighbors[neighbor_id];
    entry.info = std::make_unique<NeighborInfo>(neighbor_id, hops, returning, coords);
    entry.last_seen_call = m_call_count;
}

std::vector<NeighborInfoInterface*> NeighborManager::getNeighbors() const {
    // Evict entries not refreshed within STALE_CALLS ticks.  This keeps a
    // drone that has drifted out of RF range from anchoring the controller
    // to a stale cached position (pre-TTL, a lost drone would keep pulling
    // toward cached helpers forever, coasting past the relay chain).
    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ) {
        const uint32_t age = m_call_count - it->second.last_seen_call;
        if (age > STALE_CALLS) {
            it = m_neighbors.erase(it);
        } else {
            ++it;
        }
    }

    std::vector<NeighborInfoInterface*> neighbors;
    neighbors.reserve(m_neighbors.size());
    for (const auto& kv : m_neighbors) {
        neighbors.push_back(kv.second.info.get());
    }
    return neighbors;
}

void NeighborManager::sendToNeighbors(
    uint8_t id,
    PositionInterface* position,
    uint8_t hops_to_base_station,
    bool returning
) {
    if (!m_communication_manager || !position) {
        return;
    }

    const uint32_t this_call = m_call_count++;
    const std::vector<double> coords = position->getCoordinates();

    // Gate: honor the min-interval, then skip sends that bring no new info
    // unless the max-interval heartbeat window has elapsed (see header).
    if (m_has_last_sent) {
        const uint32_t calls_since = this_call - m_last_sent_call;
        if (calls_since < MIN_CALLS_BETWEEN_SENDS) {
            return;
        }
        if (calls_since < MAX_CALLS_BETWEEN_SENDS) {
            double dist_sq = 0.0;
            const size_t n = std::min(coords.size(), m_last_sent_coords.size());
            for (size_t i = 0; i < n; ++i) {
                const double d = coords[i] - m_last_sent_coords[i];
                dist_sq += d * d;
            }
            if (dist_sq < (NEIGHBOR_DELTA_THRESHOLD_M * NEIGHBOR_DELTA_THRESHOLD_M)) {
                return;
            }
        }
    }

    NeighborInfo info(id, hops_to_base_station, returning, coords);

    ::Packet pkt;
    pkt.type = ::PacketType::NEIGHBOR;
    pkt.src = id;
    pkt.dst = BROADCAST_ID;
    info.serialize(pkt.payload);

    m_communication_manager->send(pkt);

    m_last_sent_call = this_call;
    m_last_sent_coords = coords;
    m_has_last_sent = true;
}