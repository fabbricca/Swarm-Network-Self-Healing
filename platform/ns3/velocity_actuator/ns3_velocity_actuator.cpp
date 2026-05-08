#include "platform/ns3/velocity_actuator/ns3_velocity_actuator.h"

Ns3VelocityActuator::Ns3VelocityActuator(
    CustomMobility* mobility
) : 
    mobility(mobility)
{ }

void Ns3VelocityActuator::applyVelocity(const Vector3D acceleration, const double max_velocity) const {
    if (!mobility) {
        return;
    }
    mobility->updateVelocity(acceleration, max_velocity);
}

void Ns3VelocityActuator::brake() const {
    if (!mobility) {
        return;
    }
    mobility->brake();
}