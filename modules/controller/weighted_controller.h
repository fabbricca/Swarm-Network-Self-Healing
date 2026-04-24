#pragma once
#include "modules/controller/controller_base.h"

// Count-weighted attraction: scale each lower-hop attraction by the number
// of higher-hop neighbors and vice versa.  Σ w_i*(p_i - s) = 0 ⇒
// s = (centroid_low + centroid_high)/2, so the equilibrium sits exactly
// on the midpoint even when the lower/higher sides have different cardinality.
class WeightedController : public ControllerBase {
    public:
        using ControllerBase::ControllerBase;

    protected:
        void accumulateAttractive(
            const std::vector<NeighborInfoInterface*>& neighbors,
            uint8_t self_base_id,
            uint8_t self_hops,
            PositionInterface* self_position,
            Vector3D& F_tot
        ) override;
};
