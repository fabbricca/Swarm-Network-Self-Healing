#include "modules/controller/controller.h"
#include <iostream>
#include <unordered_map>

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

    // Group neighbors by hop count so that N drones at the same hop act as
    // a single virtual drone at their centroid.  Without this, 2 outside drones
    // pull twice as hard as 1 base, shifting equilibrium to 2/3 instead of 1/2.
    std::unordered_map<uint8_t, std::vector<std::vector<double>>> hop_groups;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        const uint8_t nh = neighbor->getHopsToBaseStation();
        if (nh != hops_from_base_station) {
            hop_groups[nh].push_back(neighbor->getPosition());
        }
    }

    Vector3D F_tot = Vector3D{0.0f, 0.0f, 0.0f};

    // Attractive force: one per hop group, toward the group centroid.
    for (const auto& [hop, positions] : hop_groups) {
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (const auto& p : positions) {
            cx += p[0];
            cy += (p.size() > 1 ? p[1] : 0.0);
            cz += (p.size() > 2 ? p[2] : 0.0);
        }
        const double n = static_cast<double>(positions.size());
        std::vector<double> centroid = {cx / n, cy / n, cz / n};
        Vector3D diff = position->distanceFromCoords(centroid);
        computeAttractiveForces(diff, F_tot);
    }

    // Repulsive force: per individual neighbor (collision avoidance).
    for (const NeighborInfoInterface* neighbor : neighbors) {
        Vector3D diff = position->distanceFromCoords(neighbor->getPosition());
        if (diff.module() < D_safe) {
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