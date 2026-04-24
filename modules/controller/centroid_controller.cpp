#include "modules/controller/centroid_controller.h"

#include <unordered_map>

void CentroidController::accumulateAttractive(
    const std::vector<NeighborInfoInterface*>& neighbors,
    uint8_t self_base_id,
    uint8_t self_hops,
    PositionInterface* self_position,
    Vector3D& F_tot
) {
    // Group different-hop neighbors by hop count (relative to self's
    // nearest base) so each hop level contributes a single attraction
    // toward its centroid.  Skip drones currently returning; collision
    // avoidance for them is handled in ControllerBase::step.
    //
    // v2 multi-base: query each neighbor's hop count relative to the base
    // WE (self) are optimizing against.  Neighbors attached to a different
    // base return UINT8_MAX and are naturally excluded.  If we have no
    // nearest base (UINT8_MAX), fall back to getMinHopsToAnyBase so the
    // drone can still form up around whatever chain exists.
    std::unordered_map<uint8_t, std::vector<std::vector<double>>> hop_groups;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t nh = (self_base_id == UINT8_MAX)
            ? neighbor->getMinHopsToAnyBase()
            : neighbor->getHopsToBase(self_base_id);
        if (nh == UINT8_MAX) continue;
        if (nh != self_hops) {
            hop_groups[nh].push_back(neighbor->getPosition());
        }
    }

    for (const auto& [hop, positions] : hop_groups) {
        (void)hop;
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
