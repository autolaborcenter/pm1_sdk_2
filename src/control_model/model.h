//
// Created by ydrml on 2019/3/15.
//

#ifndef PM1_SDK_MODEL_H
#define PM1_SDK_MODEL_H

#include "chassis_config_t.h"

// 物理模型 {较快轮的线速度, 后轮转角}
struct physical {
    float speed, rudder;
} extern const physical_zero;

// 差动模型 {左轮角速度, 右轮角速度}
struct wheels {
    float left, right;
};

// 二自由度模型 {线速度, 角速度}
struct velocity {
    float v, w;
};

/** 物理空间 -> 差动轮速空间 */
struct wheels physical_to_wheels(
    struct physical,
    const struct chassis_config_t *);

/** 差动轮速空间 -> 物理空间 */
struct physical wheels_to_physical(
    struct wheels,
    const struct chassis_config_t *);

/** 物理空间 -> 速度矢量空间 */
struct velocity physical_to_velocity(
    struct physical,
    const struct chassis_config_t *);

/** 速度矢量空间 -> 物理空间 */
struct physical velocity_to_physical(
    struct velocity,
    const struct chassis_config_t *);

/** 速度矢量空间 -> 差动轮速空间 */
struct wheels velocity_to_wheels(
    struct velocity,
    const struct chassis_config_t *);

/** 差动轮速空间 -> 速度矢量空间 */
struct velocity wheels_to_velocity(
    struct wheels,
    const struct chassis_config_t *);

#endif//PM1_SDK_MODEL_H
