#pragma once

#include <cstdint>
#include <vector>

// Broadcast destination node id.
constexpr uint8_t BROADCAST_ID = 0xFF;

// High-level packet category.
enum class PacketType : uint8_t {
  UNKNOWN = 0,
  CORE = 1,
  FLOOD = 2,
  NEIGHBOR = 3,
  UWB_BEACON = 4,
};

struct Packet {
  PacketType type = PacketType::UNKNOWN;
  uint8_t src;
  uint8_t dst;  // BROADCAST_ID = broadcast
  std::vector<uint8_t> payload;
};
