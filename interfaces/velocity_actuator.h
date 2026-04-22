#pragma once 

#include "common/vector3D.h"

class VelocityActuatorInterface {
    public:
        virtual ~VelocityActuatorInterface() = default;
        virtual void applyVelocity(const Vector3D acceleration, const double max_velocity) const = 0;
        virtual void brake() const = 0;
};