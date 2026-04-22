#include "platform/ns3/custom_mobility/custom_mobility.h"
#include <iostream>
#include <limits>
#include <cmath>

CustomMobility::CustomMobility(
    ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility
) : 
    mobility(mobility),
    previous_time_s(ns3::Simulator::Now().GetSeconds()),
    acceleration(0.0, 0.0, 0.0), // no initial acceleration
    velocity(0.0, 0.0, 0.0)  // no initial velocity
{
    if (this->mobility) {
        previous_position = this->mobility->GetPosition();
    }
}

void CustomMobility::setPosition(
    const double x,
    const double y, 
    const double z
) { 
    if (!mobility) {
        return;
    }

    mobility->SetPosition(ns3::Vector(x, y, z));
    previous_position = mobility->GetPosition();
    previous_time_s = ns3::Simulator::Now().GetSeconds();
}

void CustomMobility::update(){
    const double now_s = ns3::Simulator::Now().GetSeconds();
    const double delta_t_s = now_s - previous_time_s;
    
    // Skip update if no time has passed
    if (delta_t_s <= 0.0) {
        return;
    }
    
    double new_x, new_y, new_z;
    
    // Determine if we will reach max velocity during this time step
    const bool will_reach_max = (delta_t_max_velocity_s > 0.0) && 
                                 !std::isinf(delta_t_max_velocity_s) && 
                                 (delta_t_s > delta_t_max_velocity_s);
    
    if (!will_reach_max) {
        // Case 1: We won't reach max velocity in this time step
        // Apply standard kinematic equations: pos = pos0 + v*t + 0.5*a*t^2
        
        // First update velocity
        Vector3D new_velocity = velocity;
        new_velocity.x += acceleration.x * delta_t_s;
        new_velocity.y += acceleration.y * delta_t_s;
        new_velocity.z += acceleration.z * delta_t_s;

        // Clamp velocity if it exceeds max_velocity
        // (case in which we are already at max velocity but we are changing
        // direction) 
        if (max_velocity > 0.0 && new_velocity.module() > max_velocity) {
            new_velocity = new_velocity.unit_vector() * max_velocity;
        }
        
        // Use average velocity for position update (trapezoidal integration)
        const double avg_vx = (velocity.x + new_velocity.x) / 2.0;
        const double avg_vy = (velocity.y + new_velocity.y) / 2.0;
        const double avg_vz = (velocity.z + new_velocity.z) / 2.0;
        
        new_x = previous_position.x + avg_vx * delta_t_s;
        new_y = previous_position.y + avg_vy * delta_t_s;
        new_z = previous_position.z + avg_vz * delta_t_s;
        
        velocity = new_velocity;
    } else {
        // Case 2: We reach max velocity during this time step
        const double t_accel = delta_t_max_velocity_s;  // time accelerating
        const double t_coast = delta_t_s - t_accel;     // time at constant max velocity
        
        // Compute velocity at the transition point (when we hit max velocity)
        Vector3D v_at_max(
            velocity.x + acceleration.x * t_accel,
            velocity.y + acceleration.y * t_accel,
            velocity.z + acceleration.z * t_accel
        );

        // Clamp velocity if it exceeds max_velocity
        // (should not happen here, but keep for safety)
        if (max_velocity > 0.0 && v_at_max.module() > max_velocity) {
            v_at_max = v_at_max.unit_vector() * max_velocity;
        }
        
        // Position update in two phases:
        // Phase 1: accelerate using average velocity
        const double avg_vx_accel = (velocity.x + v_at_max.x) / 2.0;
        const double avg_vy_accel = (velocity.y + v_at_max.y) / 2.0;
        const double avg_vz_accel = (velocity.z + v_at_max.z) / 2.0;
        
        // Phase 2: coast at max velocity
        new_x = previous_position.x + avg_vx_accel * t_accel + v_at_max.x * t_coast;
        new_y = previous_position.y + avg_vy_accel * t_accel + v_at_max.y * t_coast;
        new_z = previous_position.z + avg_vz_accel * t_accel + v_at_max.z * t_coast;
        
        velocity = v_at_max;
    }

    previous_time_s = now_s;
    previous_position = ns3::Vector(new_x, new_y, new_z);
    mobility->SetPosition(previous_position);
}

std::vector<double> CustomMobility::getPosition() {
    if (!mobility) {
        return {0.0, 0.0, 0.0};
    }
    
    // update position and velocity
    // update(); 

    // retrieve current position
    previous_position = mobility->GetPosition();
    return {
        previous_position.x, 
        previous_position.y, 
        previous_position.z
    };
}

void CustomMobility::brake() {
    // Flush any accumulated motion at the current velocity up to now, then
    // zero everything so the drone holds position on subsequent ticks.
    update();
    acceleration = Vector3D(0.0, 0.0, 0.0);
    velocity = Vector3D(0.0, 0.0, 0.0);
    max_velocity = 0.0;
    delta_t_max_velocity_s = 0.0;
    previous_time_s = ns3::Simulator::Now().GetSeconds();
}

void CustomMobility::updateVelocity(const Vector3D new_acceleration, const double new_max_velocity) {
    // First, apply any pending motion from the previous acceleration
    update();

    // Store the new acceleration and max velocity
    acceleration = new_acceleration;
    max_velocity = new_max_velocity;
    delta_t_max_velocity_s = 0.0;
    
    if (acceleration.module() == 0.0 || max_velocity <= 0.0) {
        return;
    }
    
    // Check if we're already at or above max velocity
    const double current_speed = velocity.module();
    if (current_speed >= max_velocity) {
        // Already at max velocity - clamp and set delta_t to 0
        velocity = velocity.unit_vector() * max_velocity;
        delta_t_max_velocity_s = 0.0;
        return;
    }
    
    // Calculate time until we reach max velocity
    // We need to solve: |v0 + a*t|^2 = v_max^2
    // Expanding: (v0.x + a.x*t)^2 + (v0.y + a.y*t)^2 + (v0.z + a.z*t)^2 = v_max^2
    // This gives: A*t^2 + B*t + C = 0
    // where:
    //   A = |a|^2
    //   B = 2 * (v0 · a)
    //   C = |v0|^2 - v_max^2
    
    const double A = std::pow(acceleration.x, 2) + std::pow(acceleration.y, 2) + std::pow(acceleration.z, 2);
    const double B = 2.0 * (velocity.x * acceleration.x + velocity.y * acceleration.y + velocity.z * acceleration.z);
    const double C = std::pow(velocity.x, 2) + std::pow(velocity.y, 2) + std::pow(velocity.z, 2) - std::pow(max_velocity, 2);
    
    if (A == 0.0) {
        // Zero acceleration, we'll never reach max velocity through
        // acceleration 
        delta_t_max_velocity_s = std::numeric_limits<double>::infinity();
        return;
    }
    
    const double discriminant = B * B - 4.0 * A * C;
    
    if (discriminant < 0.0) {
        // No real solution - acceleration won't bring us to max velocity
        // (infeasible scenario, but handle coherently)
        delta_t_max_velocity_s = std::numeric_limits<double>::infinity();
        return;
    }
    
    const double sqrt_discriminant = std::sqrt(discriminant);
    const double t1 = (-B + sqrt_discriminant) / (2.0 * A);
    const double t2 = (-B - sqrt_discriminant) / (2.0 * A);
    
    // We want the smallest positive root
    // The two roots represent when speed increases to max_velocity and when it
    // decreases back
    
    if (t1 >= 0.0 && t2 >= 0.0) {
        // Both roots are positive - take the smaller one
        // This is when we first reach max velocity
        delta_t_max_velocity_s = std::min(t1, t2);
    } else if (t1 >= 0.0) {
        delta_t_max_velocity_s = t1;
    } else if (t2 >= 0.0) {
        delta_t_max_velocity_s = t2;
    } else {
        // Both roots are negative - we've already passed max velocity
        // (infeasible scenario, but handle coherently)
        delta_t_max_velocity_s = std::numeric_limits<double>::infinity();
    }
}