#pragma once
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "interfaces/controller.h"

enum class ControllerAlgorithm {
    Centroid,
    Weighted,
};

std::unique_ptr<ControllerInterface> makeController(
    ControllerAlgorithm algo,
    uint8_t self_id,
    float k_att,
    float k_rep,
    float d_safe,
    float v_max,
    float drone_weight_kg
);

// Returns std::nullopt on unknown names.  Accepts any case.
std::optional<ControllerAlgorithm> parseControllerAlgorithm(const std::string& name);

const char* controllerAlgorithmName(ControllerAlgorithm algo);
