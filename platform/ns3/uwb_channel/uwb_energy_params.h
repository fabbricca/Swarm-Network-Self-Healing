#pragma once

#include <cstddef>

namespace sim {

// DWM1000 (Decawave DW1000) radio energy parameters.
// Source: DW1000 User Manual v2.18, Table 6 — typical values for channel 5,
// PRF 64 MHz, 6.8 Mbps data rate, preamble length 128 symbols.
struct Dwm1000EnergyParams {
  static constexpr double TX_CURRENT_A     = 0.070;    // 70 mA
  static constexpr double RX_CURRENT_A     = 0.113;    // 113 mA
  static constexpr double IDLE_CURRENT_A   = 0.012;    // 12 mA (INIT state)
  static constexpr double SUPPLY_VOLTAGE_V = 3.3;
  static constexpr double DATA_RATE_BPS    = 6.8e6;
  static constexpr double FRAME_OVERHEAD_S = 160e-6;   // preamble + SFD + PHR

  static double frameDuration(size_t payload_bytes) {
    return FRAME_OVERHEAD_S + (payload_bytes * 8.0) / DATA_RATE_BPS;
  }

  static double txEnergy(size_t payload_bytes) {
    return frameDuration(payload_bytes) * TX_CURRENT_A * SUPPLY_VOLTAGE_V;
  }

  static double rxEnergy(size_t payload_bytes) {
    return frameDuration(payload_bytes) * RX_CURRENT_A * SUPPLY_VOLTAGE_V;
  }

  static double idleEnergy(double idle_seconds) {
    return idle_seconds * IDLE_CURRENT_A * SUPPLY_VOLTAGE_V;
  }
};

}  // namespace sim
