#include "modules/controller/controller.h"
#include <iostream>

Controller::Controller(
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

void Controller::setMissionActive(bool active) {
    mission_active = active;
}

bool Controller::isMissionActive() const {
    return mission_active;
}

void Controller::setIdleVelocity(const Vector3D& velocity) {
    idle_velocity = velocity;
}

void Controller::computeAttractiveForces(const Vector3D& diff, Vector3D& force) {
    force = force + (K_att * diff);
}

void Controller::computeRepulsiveForces(const Vector3D& diff, Vector3D& force) {
    float distance = diff.module();
    if (distance == 0) {
        // Avoid division by zero
        return; 
    }
    force = force - (K_rep / std::pow(distance, 2)) * diff.unit_vector();
}

void Controller::computeVelocityCommand(const Vector3D& force, Vector3D* new_acceleration) {
    new_acceleration->x = force.x / drone_weight_kg;
    new_acceleration->y = force.y / drone_weight_kg;
    new_acceleration->z = force.z / drone_weight_kg;
}

void Controller::step(
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
        // Mission inactive: idle behavior
        Vector3D idle_acceleration{0,0,0};
        velocity_actuator->applyVelocity(idle_acceleration, V_max);

        // Still broadcast our neighbor info while idling.
        position->retrieveCurrentPosition();
        neighbor_manager->sendToNeighbors(
            self_id,
            position,
            flooding_manager->getHopsFromBase()
        );
        return;
    }
    

    const auto neighbors = neighbor_manager->getNeighbors();
    position->retrieveCurrentPosition();
    const uint8_t hops_from_base_station = flooding_manager->getHopsFromBase();

    // Count lower-hop and higher-hop neighbors so we can weight each side by the
    // OTHER side's count.  Without this, N higher-hop drones pull N times harder
    // than a single base, shifting the equilibrium away from the midpoint.
    //
    // Scaling each lower-hop attraction by N_higher and each higher-hop attraction
    // by N_lower places the equilibrium exactly at (centroid_low + centroid_high)/2:
    //   Σ w_i*(p_i - s) = 0  ⇒  s = centroid_L/2 + centroid_H/2
    // In the common case N_lower=1, this reduces to "weigh the base as many times
    // as the higher-hop drones heard".
    uint32_t n_lower = 0, n_higher = 0;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        const uint8_t nh = neighbor->getHopsToBaseStation();
        if (nh < hops_from_base_station)      ++n_lower;
        else if (nh > hops_from_base_station) ++n_higher;
    }

    Vector3D F_tot = Vector3D{0.0f, 0.0f, 0.0f};
    for (const NeighborInfoInterface* neighbor : neighbors) {
        const uint8_t neighbor_hops = neighbor->getHopsToBaseStation();
        Vector3D diff = position->distanceFromCoords(neighbor->getPosition());
        if (neighbor_hops < hops_from_base_station) {
            // Lower-hop (toward base): weighted by number of higher-hop neighbors.
            for (uint32_t i = 0; i < n_higher; ++i) {
                computeAttractiveForces(diff, F_tot);
            }
        } else if (neighbor_hops > hops_from_base_station) {
            // Higher-hop (toward lost drones): weighted by number of lower-hop neighbors.
            for (uint32_t i = 0; i < n_lower; ++i) {
                computeAttractiveForces(diff, F_tot);
            }
        }
        if (diff.module() < D_safe) {
            // Repulsive force: per individual neighbor (collision avoidance).
            computeRepulsiveForces(diff, F_tot);
        }
    }

    // Velocity Command
    Vector3D new_acceleration(0.0, 0.0, 0.0);
    computeVelocityCommand(F_tot, &new_acceleration);
    velocity_actuator->applyVelocity(new_acceleration, V_max);

    // Broadcast to neighbors
    neighbor_manager->sendToNeighbors(
        self_id,
        position,
        hops_from_base_station
    );
}