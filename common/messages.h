#pragma once

#include <cstdint>

// Generic simulator messages shared by base and drones.
// Values are outside flooding protocol range (0..3) to simplify dispatch.
enum class SimMsgType : uint8_t {
    POS_UPDATE = 0x80,
    POS_ACK = 0x81,
    HELP_PROXY = 0x82,
};

#pragma pack(push, 1)

struct PositionUpdateMsg {
    SimMsgType type = SimMsgType::POS_UPDATE;
    uint8_t drone_id;
    uint8_t base_id;
    uint16_t seq;
    float x;
    float y;
    float z;
};

struct PositionAckMsg {
    SimMsgType type = SimMsgType::POS_ACK;
    uint8_t base_id;
    uint8_t drone_id;
    uint16_t seq;

    // Include base station data in the same spirit as NeighborInfo:
    // - hops_to_base_station (base is 0)
    // - coordinates (3D)
    uint8_t base_hops_to_base_station = 0;
    double x;
    double y;
    double z;
};

struct HelpProxyMsg {
    SimMsgType type = SimMsgType::HELP_PROXY;
    uint8_t requester_id;
    uint8_t base_id;
};

// UWB beacon broadcast by anchors at a fixed rate.
// Drones use the timestamp + known propagation speed to compute
// ToF-based range for trilateration.
struct UwbBeaconMsg {
    uint8_t anchor_id;
    double  tx_timestamp_s;  // Simulation time at transmission
    double  x;               // Anchor position (known, fixed)
    double  y;
    double  z;
};

#pragma pack(pop)

static_assert(sizeof(HelpProxyMsg) == 3, "HelpProxyMsg must be packed");
static_assert(sizeof(PositionUpdateMsg) == 17, "PositionUpdateMsg must be packed");
static_assert(sizeof(PositionAckMsg) == 30, "PositionAckMsg must be packed");
static_assert(sizeof(UwbBeaconMsg) == 33, "UwbBeaconMsg must be packed");
