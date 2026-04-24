#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <cstring>
#include <iostream>

#include "interfaces/flood_manager.h"

#include "modules/flood/flood_messages.h"

class FloodManager : public FloodManagerInterface {
 public:
    FloodManager(
        uint8_t self_id,
        CommunicationManagerInterface& communication_manager,
        std::function<bool(uint8_t)> is_base_reachable = {}
    );

    void onPacketReceived(const ::Packet& pkt) override;

    // v2: register every base this drone should track.  Floods from
    // unregistered bases are silently dropped.  Each base has its own
    // per-flood/best-hop state so cross-base hop counts never mix.
    void registerBase(uint8_t base_id);

    // Base-side only: set the id this FloodManager stamps on discovery/
    // report messages when it originates them.  Drones use registerBase()
    // instead (they never originate floods).
    void setSelfBaseId(uint8_t base_id);

    uint8_t getHopsFromBase(uint8_t base_id) const override;
    uint8_t getNearestBaseId() const override;
    uint8_t getNearestBaseHops() const override;
    std::vector<std::pair<uint8_t, uint8_t>> getPerBaseHops() const override;

 private:
    struct PerBase {
        std::unordered_map<uint16_t, uint8_t> best_hop_per_flood;
        std::unordered_map<uint16_t, std::unordered_map<uint8_t, uint8_t>> best_report_seen;
        std::unordered_set<uint16_t> seen_floods;
    };

    uint8_t self_id;
    uint8_t self_base_id = 0;          // only meaningful for base-side managers
    CommunicationManagerInterface& communication_manager;
    // Callable: is_base_reachable(base_id) -> bool.  Lets getHopsFromBase()
    // short-circuit to 1 when the drone has a fresh direct ACK from that
    // base (even if no flood has converged yet), and prevents reporting a
    // stale hop=1 for a base whose ACK timed out.
    std::function<bool(uint8_t)> is_base_reachable;

    std::unordered_map<uint8_t, PerBase> per_base;

    void startFlood(uint16_t flood_id) override;

    void handleStart(const FloodStartMsg& msg);
    void handleDiscovery(const FloodDiscoveryMsg& msg);
    void handleReport(const FloodReportMsg& msg);

    ::Packet createReportMsg(uint8_t base_id, uint16_t flood_id, uint8_t initiator_id, uint8_t candidate_hop);
    ::Packet createDiscoveryMsg(uint8_t base_id, uint16_t flood_id, uint8_t initiator_id, uint8_t hop_to_base);

    static bool decodeStart(const ::Packet& pkt, FloodStartMsg& msg);
    static bool decodeDiscovery(const ::Packet& pkt, FloodDiscoveryMsg& msg);
    static bool decodeReport(const ::Packet& pkt, FloodReportMsg& msg);
};
