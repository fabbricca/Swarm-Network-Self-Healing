#pragma once
#include <cstdint>
#include <utility>
#include <vector>
#include "communication_manager.h"

class FloodManagerInterface {
    public:
        virtual ~FloodManagerInterface() = default;
        virtual void onPacketReceived(const ::Packet& pkt) = 0;

        // v2 roaming: drones track hop counts for every base they hear from.
        // Callers that don't care which base is best can use
        // getNearestBaseId()/getNearestBaseHops().
        virtual uint8_t getHopsFromBase(uint8_t base_id) const = 0;
        virtual uint8_t getNearestBaseId() const = 0;       // UINT8_MAX if none reachable
        virtual uint8_t getNearestBaseHops() const = 0;     // UINT8_MAX if none reachable

        // Snapshot of every (base_id, hops) pair this node currently knows
        // about.  Used by NeighborManager to broadcast our per-base state.
        virtual std::vector<std::pair<uint8_t, uint8_t>> getPerBaseHops() const = 0;

    private:
        virtual void startFlood(uint16_t flood_id) = 0;
};
