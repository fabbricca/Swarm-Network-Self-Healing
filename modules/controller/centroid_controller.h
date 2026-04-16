#pragma once
#include "modules/controller/controller_base.h"

// Groups neighbors by hop count and applies one attractive force per
// hop-group centroid.  Keeps a multi-drone side from dominating the balance:
// N drones at the same hop pull as a single virtual drone at their mean
// position, so the equilibrium lands on the midpoint between sides.
class CentroidController : public ControllerBase {
    public:
        using ControllerBase::ControllerBase;

    protected:
        void accumulateAttractive(
            const std::vector<NeighborInfoInterface*>& neighbors,
            uint8_t self_hops,
            PositionInterface* self_position,
            Vector3D& F_tot
        ) override;
};
