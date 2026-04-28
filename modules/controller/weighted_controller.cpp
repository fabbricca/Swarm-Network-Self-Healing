#include "modules/controller/weighted_controller.h"

#include <cstdint>

void WeightedController::accumulateAttractive(
    const std::vector<NeighborInfoInterface*>& neighbors,
    uint8_t self_base_id,
    uint8_t self_hops,
    PositionInterface* self_position,
    Vector3D& F_tot
) {
    // Count lower/higher-hop neighbors relative to OUR nearest base so each
    // side can be scaled by the opposing side's cardinality.
    //
    // v2 multi-base: every per-neighbor hop query is against self_base_id so
    // neighbors attached to other bases don't bias the count.  If we have
    // no nearest base (UINT8_MAX), fall back to min-hops-to-any-base so
    // the drone still gets some gradient signal.
    //
    // A neighbor with no path to OUR base: if it has a path to some other
    // base, it belongs to that swarm and we skip it.  If it has no path to
    // ANY base, it is genuinely lost — treat it as higher-hop so it acts
    // as an outward attractor.  Without this, n_higher collapses to 0 when
    // the only "outward" neighbor is the lost drone, the cross-product
    // weight zeroes the inward pull, F_tot ≈ 0, and the helper brakes at
    // the coverage boundary instead of holding the midpoint.
    auto hop_to_our_base = [&](const NeighborInfoInterface* n) -> uint8_t {
        uint8_t h = (self_base_id == UINT8_MAX)
            ? n->getMinHopsToAnyBase()
            : n->getHopsToBase(self_base_id);
        if (h == UINT8_MAX && n->getMinHopsToAnyBase() == UINT8_MAX) {
            // Genuinely lost — sentinel above any real hop count.
            return UINT8_MAX;
        }
        return h;
    };

    uint32_t n_lower = 0, n_higher = 0;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t nh = hop_to_our_base(neighbor);
        // UINT8_MAX from a neighbor still attached to another base means
        // "skip"; UINT8_MAX from a fully lost neighbor means "higher hop".
        if (nh == UINT8_MAX && neighbor->getMinHopsToAnyBase() != UINT8_MAX) continue;
        if (nh < self_hops)      ++n_lower;
        else if (nh > self_hops) ++n_higher;
    }

    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t neighbor_hops = hop_to_our_base(neighbor);
        if (neighbor_hops == UINT8_MAX && neighbor->getMinHopsToAnyBase() != UINT8_MAX) continue;
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
