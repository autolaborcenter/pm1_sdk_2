//
// Created by ydrml on 2021/3/31.
//

#ifndef PM1_SDK_2_CHASSIS_MODEL_HH
#define PM1_SDK_2_CHASSIS_MODEL_HH

extern "C" {
#include "control_model/optimization.h"
}

#include <chrono>

namespace autolabor::pm1 {
    class chassis_model_t {

        chassis_config_t _chassis_config = default_config;
        optimizer _optimizer = default_optimizer;

    public:
        constexpr static auto CONTROL_PERIOD = std::chrono::milliseconds(20);

        [[nodiscard]] physical optimize(const physical &target, const physical &current) const;

        [[nodiscard]] wheels to_wheels(const physical &) const;

        [[nodiscard]] velocity to_velocity(const physical &) const;

        [[nodiscard]] physical from_wheels(const wheels &) const;

        [[nodiscard]] physical from_velocity(const velocity &) const;
    };
}// namespace autolabor::pm1

#endif//PM1_SDK_2_CHASSIS_MODEL_HH
