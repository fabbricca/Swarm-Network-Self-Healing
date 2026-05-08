#include "modules/controller/controller_factory.h"

#include <algorithm>
#include <cctype>

#include "modules/controller/centroid_controller.h"
#include "modules/controller/weighted_controller.h"

std::unique_ptr<ControllerInterface> makeController(
    ControllerAlgorithm algo,
    uint8_t self_id,
    float k_att,
    float k_rep,
    float d_safe,
    float v_max,
    float drone_weight_kg
) {
    switch (algo) {
        case ControllerAlgorithm::Centroid:
            return std::make_unique<CentroidController>(
                self_id, k_att, k_rep, d_safe, v_max, drone_weight_kg);
        case ControllerAlgorithm::Weighted:
            return std::make_unique<WeightedController>(
                self_id, k_att, k_rep, d_safe, v_max, drone_weight_kg);
    }
    return nullptr;
}

std::optional<ControllerAlgorithm> parseControllerAlgorithm(const std::string& name) {
    std::string lower;
    lower.reserve(name.size());
    for (char c : name) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    if (lower == "centroid") return ControllerAlgorithm::Centroid;
    if (lower == "weighted") return ControllerAlgorithm::Weighted;
    return std::nullopt;
}

const char* controllerAlgorithmName(ControllerAlgorithm algo) {
    switch (algo) {
        case ControllerAlgorithm::Centroid: return "centroid";
        case ControllerAlgorithm::Weighted: return "weighted";
    }
    return "unknown";
}
