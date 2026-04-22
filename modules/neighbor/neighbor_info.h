#pragma once
#include <cstdint>
#include <vector>
#include <cstring>
#include <stdexcept>

#include "interfaces/neighbor_info.h"

enum class NeighborMsgType : uint8_t {
    NEIGHBOR_INFO = 10,
};

class NeighborInfo : public NeighborInfoInterface {
    public: 
        NeighborInfo(
            uint8_t id,
            uint8_t hops,
            bool returning,
            const std::vector<double>& coordinates
        );

        std::vector<double> getPosition() const override;
        uint8_t getHopsToBaseStation() const override;
        bool getIsReturning() const override;
        void serialize(std::vector<uint8_t>& out_payload) const override;
        void deserialize(const std::vector<uint8_t>& in_payload) override;

        // Payload bit layout for the flags byte.
        static constexpr uint8_t FLAG_RETURNING = 0x01;

    private:
        uint8_t neighbor_id;
        uint8_t hops_from_base_station;
        bool is_returning;
        std::vector<double> position;
};