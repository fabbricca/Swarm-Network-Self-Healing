#pragma once
#include "interfaces/velocity_actuator.h"
#include "platform/ns3/custom_mobility/custom_mobility.h"

class Ns3VelocityActuator : public VelocityActuatorInterface {
    public:
        Ns3VelocityActuator(CustomMobility* mobility);
        void applyVelocity(const Vector3D acceleration, const double max_velocity) const override;
        void brake() const override;

    private: 
        CustomMobility* mobility;
};