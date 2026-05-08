#pragma once
#include <vector>

#include "ns3/core-module.h"
#include "ns3/constant-position-mobility-model.h"
#include "common/vector3D.h"

class CustomMobility {
    public:
        CustomMobility(ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility);
        void setPosition(const double x, const double y, const double z); 
        void update();
        std::vector<double> getPosition();
        void updateVelocity(const Vector3D acceleration, const double max_velocity);
        void brake();
        
    private:
        ns3::Ptr<ns3::MobilityModel> mobility;
        ns3::Vector previous_position;
        double previous_time_s = 0.0;  // NS-3 simulation time (seconds)
        double delta_t_max_velocity_s = 0.0;
        double max_velocity = 0.0;
        Vector3D acceleration;
        Vector3D velocity;
    };