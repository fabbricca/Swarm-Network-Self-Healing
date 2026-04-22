#include "modules/controller/centroid_controller.h"

#include <unordered_map>

void CentroidController::accumulateAttractive(
    const std::vector<NeighborInfoInterface*>& neighbors,
    uint8_t self_hops,
    PositionInterface* self_position,
    Vector3D& F_tot
) {
    // Group different-hop neighbors by hop count so each hop level contributes
    // a single attraction toward its centroid — stops N drones at one hop
    // from pulling N times harder than a single drone on the other side.
    // Skip drones currently returning: they are not valid formation anchors
    // (they are actively retreating toward base).  Collision-avoid repulsion
    // is applied in ControllerBase::step(), so they are still respected for
    // safety.
    std::unordered_map<uint8_t, std::vector<std::vector<double>>> hop_groups;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t nh = neighbor->getHopsToBaseStation();
        if (nh != self_hops) {
            hop_groups[nh].push_back(neighbor->getPosition());
        }
    }

    for (const auto& [hop, positions] : hop_groups) {
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (const auto& p : positions) {
            cx += p[0];
            cy += (p.size() > 1 ? p[1] : 0.0);
            cz += (p.size() > 2 ? p[2] : 0.0);
        }
        const double n = static_cast<double>(positions.size());
        std::vector<double> centroid = {cx / n, cy / n, cz / n};
        Vector3D diff = self_position->distanceFromCoords(centroid);
        computeAttractiveForces(diff, F_tot);
    }
}
