#include "modules/neighbor/neighbor_info.h"

NeighborInfo::NeighborInfo(
    uint8_t id,
    uint8_t hops,
    bool returning,
    const std::vector<double>& coordinates
) :
    neighbor_id(id),
    hops_from_base_station(hops),
    is_returning(returning),
    position(coordinates)
{ }

std::vector<double> NeighborInfo::getPosition() const {
    return position;
}

uint8_t NeighborInfo::getHopsToBaseStation() const {
    return hops_from_base_station;
}

bool NeighborInfo::getIsReturning() const {
    return is_returning;
}

void NeighborInfo::serialize(std::vector<uint8_t>& out_payload) const {
    const size_t position_size = position.size() * sizeof(double);
    out_payload.resize(3 + position_size);

    out_payload[0] = neighbor_id;
    out_payload[1] = hops_from_base_station;
    out_payload[2] = static_cast<uint8_t>(is_returning ? FLAG_RETURNING : 0);
    if (position_size > 0) {
        std::memcpy(&out_payload[3], position.data(), position_size);
    }
}

void NeighborInfo::deserialize(const std::vector<uint8_t>& in_payload) {
    if (in_payload.size() < 3) {
        throw std::invalid_argument("Payload too small to deserialize NeighborInfo");
    }

    const size_t position_size = in_payload.size() - 3;
    position.resize(position_size / sizeof(double));

    neighbor_id = in_payload[0];
    hops_from_base_station = in_payload[1];
    is_returning = (in_payload[2] & FLAG_RETURNING) != 0;
    if (position_size > 0) {
        std::memcpy(position.data(), &in_payload[3], position_size);
    }
}