#include "modules/dispatch/dispatch_manager.h"

void DispatchManager::setFloodManager(FloodManagerInterface* flood_manager) {
  m_flood_manager = flood_manager;
}

void DispatchManager::setNeighborManager(NeighborManagerInterface* neighbor_manager) {
  m_neighbor_manager = neighbor_manager;
}

void DispatchManager::setUwbRangingManager(UwbRangingManagerInterface* uwb_ranging_manager) {
  m_uwb_ranging_manager = uwb_ranging_manager;
}

void DispatchManager::setFallbackHandler(FallbackHandler handler) {
  m_fallback_handler = std::move(handler);
}

void DispatchManager::handlePacket(const ::Packet& pkt) const {
  switch (pkt.type) {
    case ::PacketType::FLOOD:
      if (m_flood_manager) {
        m_flood_manager->onPacketReceived(pkt);
      }
      return;

    case ::PacketType::NEIGHBOR:
      if (m_neighbor_manager) {
        m_neighbor_manager->onPacketReceived(pkt);
      }
      return;

    case ::PacketType::UWB_BEACON:
      if (m_uwb_ranging_manager) {
        m_uwb_ranging_manager->onPacketReceived(pkt);
      }
      return;

    case ::PacketType::CORE:
    case ::PacketType::UNKNOWN:
    default:
      break;
  }

  if (m_fallback_handler) {
    m_fallback_handler(pkt);
  }
}