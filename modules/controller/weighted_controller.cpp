#include "modules/controller/weighted_controller.h"

#include <cstdint>

void WeightedController::accumulateAttractive(
    const std::vector<NeighborInfoInterface*>& neighbors,
    uint8_t self_hops,
    PositionInterface* self_position,
    Vector3D& F_tot
) {
    // Count lower/higher-hop neighbors first so each side can be scaled by
    // the OTHER side's cardinality.  Without this, N higher-hop drones pull
    // N times harder than a single base, shifting the equilibrium off the
    // midpoint.  In the common case N_lower=1 this reduces to "weigh the
    // base as many times as the higher-hop drones heard".
    // Skip drones currently returning: they are not valid formation anchors.
    uint32_t n_lower = 0, n_higher = 0;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t nh = neighbor->getHopsToBaseStation();
        if (nh < self_hops)      ++n_lower;
        else if (nh > self_hops) ++n_higher;
    }

    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t neighbor_hops = neighbor->getHopsToBaseStation();
        Vector3D diff = self_position->distanceFromCoords(neighbor->getPosition());
        if (neighbor_hops < self_hops) {
            for (uint32_t i = 0; i < n_higher; ++i) {
                computeAttractiveForces(diff, F_tot);
            }
        } else if (neighbor_hops > self_hops) {
            for (uint32_t i = 0; i < n_lower; ++i) {
                computeAttractiveForces(diff, F_tot);
            }
        }
    }
}
