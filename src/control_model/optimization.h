﻿//
// Created by ydrml on 2019/3/20.
//

#ifndef PM1_SDK_OPTIMIZATION_H
#define PM1_SDK_OPTIMIZATION_H

#include "model.h"

struct optimizer {
    float
        ratio_tail_physical_speed,
        acceleration,
        period;
};
extern const struct optimizer default_optimizer;

/**
 * 控制量优化
 * @param target  目标控制量
 *                从遥控器或其他上层控制系统计算得来，保证任意取值范围
 * @param current 当前控制量
 *                当前后轮转角应当测得，当前速度若不好测量，可使用上一次的目标
 * @return 当前可执行的控制量，速度经过优化，后轮转角等于当前后轮转角
 */
struct physical optimize(struct physical target,
                         struct physical current,
                         const struct optimizer *,
                         const struct chassis_config_t *);

#endif//PM1_SDK_OPTIMIZATION_H
