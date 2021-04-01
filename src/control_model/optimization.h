//
// Created by ydrml on 2019/3/20.
//

#ifndef PM1_SDK_OPTIMIZATION_H
#define PM1_SDK_OPTIMIZATION_H

#include "model.h"

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
                         float optimize_width,
                         float stepover);

/** 在速度空间中限速 */
void limit_in_velocity(struct physical *,
                       float max_v,
                       float max_w,
                       const struct chassis_config_t *);

/** 在物理模型空间中限速 */
void limit_in_physical(struct physical *,
                       float);

/**
 * 基于结构的限速
 * @param k 允许的甩尾速度与轮速的比
 */
void limit_by_struct(struct physical *,
                     float k,
                     const struct chassis_config_t *);

#endif //PM1_SDK_OPTIMIZATION_H
