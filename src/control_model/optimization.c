//
// Created by ydrml on 2019/3/20.
//

#include "optimization.h"
#include <math.h>

const struct optimizer default_optimizer = {1.0f, 0.8f, 0.02f};

struct physical optimize(struct physical target,
                         struct physical current,
                         const struct optimizer *parameter,
                         const struct chassis_config_t *chassis) {
    // 基于现象的限速
    struct velocity temp = physical_to_velocity(target, chassis);
    float max_w = parameter->ratio_tail_physical_speed * target.speed / chassis->length;
    target.speed /= fmaxf(1, fabsf(temp.w / max_w));
    // 基于结构的限速
    struct physical result = {
        isnan(target.rudder) ? 0 : cosf(target.rudder - current.rudder) * target.speed,
        current.rudder};
    // 动态调速
    float stepover = parameter->acceleration * parameter->period;
    result.speed = result.speed > current.speed
                       ? fminf(current.speed + stepover, result.speed)
                       : fmaxf(current.speed - stepover, result.speed);
    return result;
}
