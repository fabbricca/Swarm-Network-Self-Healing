#pragma once
#include <cstdint>

#include "common/vector3D.h"
#include "interfaces/flood_manager.h"
#include "interfaces/velocity_actuator.h"
#include "interfaces/neighbor_manager.h"
#include "interfaces/position.h"

class ControllerInterface {
    public:
        virtual ~ControllerInterface() = default;

        virtual void setMissionActive(bool active) = 0;
        virtual bool isMissionActive() const = 0;
        virtual void setIdleVelocity(const Vector3D& velocity) = 0;

        virtual void step(
            FloodManagerInterface* flooding_manager,
            VelocityActuatorInterface* velocity_actuator,
            NeighborManagerInterface* neighbor_manager,
            PositionInterface* position
        ) = 0;
};
