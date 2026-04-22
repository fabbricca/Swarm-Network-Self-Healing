#pragma once
#include <cstdint>
#include <vector> 

#include "interfaces/neighbor_info.h"
#include "interfaces/communication_manager.h"
#include "interfaces/position.h"

class NeighborManagerInterface {
    public:
        virtual ~NeighborManagerInterface() = default;
        virtual void onPacketReceived(const ::Packet& pkt) = 0;
        virtual std::vector<NeighborInfoInterface*> getNeighbors() const = 0;
        virtual void sendToNeighbors(
            uint8_t id,
            PositionInterface* position,
            uint8_t hops_to_base_station,
            bool returning
        ) = 0;
};