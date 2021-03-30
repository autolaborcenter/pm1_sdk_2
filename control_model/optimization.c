//
// Created by ydrml on 2019/3/20.
//

#include <math.h>
#include "optimization.h"

struct physical optimize(struct physical target,
                         struct physical current,
                         float optimize_width,
                         float acceleration) {
    struct physical result = {0, current.rudder};
    // 等待后轮转动
    if (!isnan(target.rudder))
        result.speed = target.speed * fmaxf(0, 1 - fabsf(target.rudder - current.rudder) / optimize_width);
    // 动态调速
    result.speed = result.speed > current.speed
                   ? fminf(current.speed + acceleration, result.speed)
                   : fmaxf(current.speed - acceleration, result.speed);
    return result;
}

void limit_in_velocity(struct physical *physical,
                       float max_v,
                       float max_w,
                       const struct chassis_config_t *chassis) {
    struct velocity temp = physical_to_velocity(*physical, chassis);
    physical->speed /= fmaxf(1, fmaxf(fabsf(temp.v / max_v),
                                      fabsf(temp.w / max_w)));
}

void limit_in_physical(struct physical *physical,
                       float max_wheel_speed) {
    physical->speed /= fmaxf(1, fabsf(physical->speed / max_wheel_speed));
}

void limit_by_struct(struct physical *physical,
                     float k,
                     const struct chassis_config_t *chassis) {
    struct velocity temp = physical_to_velocity(*physical, chassis);
    float max_w = k * physical->speed / chassis->length;
    physical->speed /= fmaxf(1, fabsf(temp.w / max_w));
}
