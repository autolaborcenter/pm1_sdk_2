﻿//
// Created by ydrml on 2019/3/15.
//

#include <math.h>
#include "model.h"

struct wheels physical_to_wheels(
    struct physical physical,
    const struct chassis_config_t *config) {
    struct wheels result;
    
    if (physical.speed == 0) {
        // 对于舵轮来说是奇点，无法恢复
        result.left = 0;
        result.right = 0;
        
    } else if (physical.rudder == 0) {
        // 直走
        result.left = physical.speed / config->left_radius;
        result.right = physical.speed / config->right_radius;
        
    } else {
        // 圆弧
        float r = -config->length / tanf(physical.rudder);
        
        if (physical.rudder > 0) {
            // 右转，左轮线速度快
            float k = (r + config->width / 2) / (r - config->width / 2);
            
            result.left = physical.speed / config->left_radius;
            result.right = physical.speed / config->right_radius * k;
            
        } else {
            // 左转，右轮线速度快
            float k = (r - config->width / 2) / (r + config->width / 2);
            
            result.left = physical.speed / config->left_radius * k;
            result.right = physical.speed / config->right_radius;
        }
    }
    
    return result;
}

struct physical wheels_to_physical(
    struct wheels wheels,
    const struct chassis_config_t *config) {
    struct physical result;
    
    wheels.left *= config->left_radius;
    wheels.right *= config->right_radius;
    
    float left = fabsf(wheels.left),
        right = fabsf(wheels.right);
    
    if (left == right) {
        // 绝对值相等（两条对角线）
        if (left == 0) {
            // 奇点
            result.speed = 0;
            result.rudder = NAN;
            
        } else if (wheels.left > wheels.right) {
            // 副对角线
            result.speed = left;
            result.rudder = +pi_f / 2;
            
        } else if (wheels.left < wheels.right) {
            // 副对角线
            result.speed = right;
            result.rudder = -pi_f / 2;
            
        } else {
            // 主对角线
            result.speed = wheels.left;
            result.rudder = 0;
        }
        
    } else {
        if (left > right) {
            // 右转，左轮速度快
            float k = wheels.right / wheels.left;
            float r = config->width / 2 * (k + 1) / (k - 1);
            
            result.speed = wheels.left;
            result.rudder = -atanf(config->length / r);
            
        } else {
            // 左转，右轮速度快
            float k = wheels.left / wheels.right;
            float r = config->width / 2 * (1 + k) / (1 - k);
            
            result.speed = wheels.right;
            result.rudder = -atanf(config->length / r);
        }
    }
    
    return result;
}

struct velocity physical_to_velocity(
    struct physical physical,
    const struct chassis_config_t *config) {
    return wheels_to_velocity(physical_to_wheels(physical, config), config);
}

struct physical velocity_to_physical(
    struct velocity velocity,
    const struct chassis_config_t *config) {
    return wheels_to_physical(velocity_to_wheels(velocity, config), config);
}

struct wheels velocity_to_wheels(
    struct velocity velocity,
    const struct chassis_config_t *config) {
    struct wheels result = {
        (velocity.v - config->width / 2 * velocity.w) / config->left_radius,
        (velocity.v + config->width / 2 * velocity.w) / config->right_radius
    };
    return result;
}

struct velocity wheels_to_velocity(
    struct wheels wheels,
    const struct chassis_config_t *config) {
    struct velocity result = {
        config->left_radius * (wheels.right + wheels.left) / 2,
        config->right_radius * (wheels.right - wheels.left) / config->width
    };
    return result;
}

void limit_in_velocity(struct physical *data,
                       float max_v,
                       float max_w,
                       const struct chassis_config_t *chassis) {
    struct velocity temp = physical_to_velocity(*data, chassis);
    data->speed /= fmaxf(1, fmaxf(fabsf(temp.v / max_v),
                                  fabsf(temp.w / max_w)));
}

void limit_in_physical(struct physical *data, float max_wheel_speed) {
    data->speed /= fmaxf(1, fabsf(data->speed / max_wheel_speed));
}
