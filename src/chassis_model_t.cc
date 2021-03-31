//
// Created by ydrml on 2021/3/31.
//

#include "chassis_model_t.hh"

namespace autolabor::pm1 {
    physical chassis_model_t::optimize(const physical &target, const physical &current) const {
        auto stepover = _acceleration * std::chrono::duration<float, std::ratio<1>>(CONTROL_PERIOD).count();
        auto optimized = ::optimize(target, current, _optimize_width, stepover);
        limit_by_struct(&optimized, _ratio_tail_physical_speed, &_chassis_config);
        return optimized;
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
}
