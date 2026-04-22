#pragma once
#include <cstdint>
#include <vector>

class NeighborInfoInterface {
    public:
        virtual ~NeighborInfoInterface() = default;
        virtual std::vector<double> getPosition() const = 0;
        virtual uint8_t getHopsToBaseStation() const = 0;
        virtual bool getIsReturning() const = 0;
        virtual void serialize(std::vector<uint8_t>& out_payload) const = 0;
        virtual void deserialize(const std::vector<uint8_t>& in_payload) = 0;
};