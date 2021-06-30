//
// Created by ydrml on 2021/3/31.
//

#include "chassis_model_t.hh"

extern "C" {
#include "control_model/motor_map.h"
}

namespace autolabor::pm1 {
    physical chassis_model_t::optimize(const physical &target, const physical &current) const {
        return ::optimize(target, current, &_optimizer, &_chassis_config);
    }

    wheels chassis_model_t::to_wheels(const physical &p) const {
        return physical_to_wheels(p, &_chassis_config);
    }

    velocity chassis_model_t::to_velocity(const physical &p) const {
        return physical_to_velocity(p, &_chassis_config);
    }

    physical chassis_model_t::from_wheels(const wheels &w) const {
        return wheels_to_physical(w, &_chassis_config);
    }

    physical chassis_model_t::from_velocity(const velocity &v) const {
        return velocity_to_physical(v, &_chassis_config);
    }

    std::chrono::milliseconds chassis_model_t::period() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<float, std::ratio<1>>(_optimizer.period));
    }

    odometry_delta_t<> chassis_model_t::to_delta(int l_, int r_) const {
        const auto l = RAD_OF(l_, default_wheel_k) * _chassis_config.left_radius,
                   r = RAD_OF(r_, default_wheel_k) * _chassis_config.right_radius,
                   s = (r + l) / 2,
                   a = (r - l) / _chassis_config.width;
        return odometry_delta_t<>::from_velocity(s, a);
    }

    odometry_delta_t<> chassis_model_t::to_delta(physical p) const {
        const auto velocity = to_velocity(p);
        return odometry_delta_t<>::from_velocity(
            velocity.v * _optimizer.period,
            velocity.w * _optimizer.period);
    }

}// namespace autolabor::pm1
