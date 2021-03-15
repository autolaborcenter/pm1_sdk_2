cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

add_library(control_model
        pi.h

        model.h
        model.c

        chassis_config_t.h
        chassis_config_t.c

        motor_map.h

        optimization.h
        optimization.c
        )
