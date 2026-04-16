#include "modules/controller/controller_base.h"

#include <cmath>

ControllerBase::ControllerBase(
    uint8_t self_id,
    float K_att_value,
    float K_rep_value,
    float D_safe,
    float V_max,
    float drone_weight_kg
) :
    self_id(self_id),
    K_att(K_att_value > 0.0 ? K_att_value : DEFAULT_K_ATT),
    K_rep(K_rep_value > 0.0 ? K_rep_value : DEFAULT_K_REP),
    D_safe(D_safe > 0.0 ? D_safe : DEFAULT_D_SAFE),
    V_max(V_max > 0.0 ? V_max : DEFAULT_V_MAX),
    drone_weight_kg(drone_weight_kg > 0.0 ? drone_weight_kg : DEFAULT_DRONE_WEIGHT_KG)
{ }

void ControllerBase::setMissionActive(bool active) {
    mission_active = active;
}

bool ControllerBase::isMissionActive() const {
    return mission_active;
}

void ControllerBase::setIdleVelocity(const Vector3D& velocity) {
    idle_velocity = velocity;
}

void ControllerBase::computeAttractiveForces(const Vector3D& diff, Vector3D& force) {
    force = force + (K_att * diff);
}

void ControllerBase::computeRepulsiveForces(const Vector3D& diff, Vector3D& force) {
    float distance = diff.module();
    if (distance == 0) {
        return;
    }
    force = force - (K_rep / std::pow(distance, 2)) * diff.unit_vector();
}

void ControllerBase::computeVelocityCommand(const Vector3D& force, Vector3D* new_acceleration) {
    new_acceleration->x = force.x / drone_weight_kg;
    new_acceleration->y = force.y / drone_weight_kg;
    new_acceleration->z = force.z / drone_weight_kg;
}

void ControllerBase::step(
    FloodManagerInterface* flooding_manager,
    VelocityActuatorInterface* velocity_actuator,
    NeighborManagerInterface* neighbor_manager,
    PositionInterface* position
) {
    if (!velocity_actuator) {
        return;
    }
    if (!flooding_manager || !neighbor_manager || !position) {
        return;
    }
    if (!mission_active) {
        Vector3D idle_acceleration{0, 0, 0};
        velocity_actuator->applyVelocity(idle_acceleration, V_max);

        // Silence NEIGHBOR only when we're in base coverage (hops==1): an
        // in-coverage idle drone plays no role in the formation task and
        // costs a broadcast per tick for nothing.  Lost idle drones (hops>1
        // or UINT8_MAX) still advertise so helpers can pull toward them.
        const uint8_t hops = flooding_manager->getHopsFromBase();
        if (hops != 1) {
            position->retrieveCurrentPosition();
            neighbor_manager->sendToNeighbors(self_id, position, hops);
        }
        return;
    }

    const auto neighbors = neighbor_manager->getNeighbors();
    position->retrieveCurrentPosition();
    const uint8_t hops_from_base_station = flooding_manager->getHopsFromBase();

    Vector3D F_tot{0.0f, 0.0f, 0.0f};
    accumulateAttractive(neighbors, hops_from_base_station, position, F_tot);

    for (const NeighborInfoInterface* neighbor : neighbors) {
        Vector3D diff = position->distanceFromCoords(neighbor->getPosition());
        if (diff.module() < D_safe) {
            computeRepulsiveForces(diff, F_tot);
        }
    }

    Vector3D new_acceleration(0.0, 0.0, 0.0);
    computeVelocityCommand(F_tot, &new_acceleration);
    velocity_actuator->applyVelocity(new_acceleration, V_max);

    neighbor_manager->sendToNeighbors(
        self_id,
        position,
        hops_from_base_station
    );
}
