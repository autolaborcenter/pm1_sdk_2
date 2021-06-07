cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pm1_sdk_2 LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 20)

include_directories(src)

add_library(control_model
        src/control_model/pi.h

        src/control_model/model.h
        src/control_model/model.c

        src/control_model/chassis_config_t.h
        src/control_model/chassis_config_t.c

        src/control_model/motor_map.h

        src/control_model/optimization.h
        src/control_model/optimization.c)

add_library(pm1
        src/autocan/protocol.h
        src/autocan/pm1.h
        src/autocan/parse.cpp

        src/chassis_t.hh
        src/chassis_t.cc

        src/chassis_model_t.hh
        src/chassis_model_t.cc)
target_link_libraries(pm1 control_model)

find_package(Threads REQUIRED)

if (WIN32)

elseif (UNIX)

# 云台舵机

add_executable(servo_test test/servo_test.cpp
        app/serial_linux.h
        app/serial_linux.cpp)

# 罗技 g29

add_library(g29
        src/g29/steering_t.hh
        src/g29/steering_t.cc)
target_link_libraries(g29 control_model)

add_executable(g29_test test/g29_display.cpp)
target_link_libraries(g29_test g29)

add_executable(g29_with_udp app/g29_with_udp.cpp)
target_link_libraries(g29_with_udp g29)

# PM1

add_library(pm1_linux
        app/pm1_driver_common.h
        app/pm1_driver_common_linux.cpp
        app/serial_linux.h
        app/serial_linux.cpp)
target_link_libraries(pm1_linux pm1 ${CMAKE_THREAD_LIBS_INIT})

add_executable(pm1_with_g29 test/pm1_with_g29.cpp)
target_link_libraries(pm1_with_g29 pm1_linux g29)

add_executable(pm1_with_console test/pm1_with_console.cpp)
target_link_libraries(pm1_with_console pm1_linux)

add_executable(pm1_with_udp app/pm1_with_udp.cpp)
target_link_libraries(pm1_with_udp pm1_linux)

# ---

EXECUTE_PROCESS(COMMAND uname -m OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE architecture)
if (architecture STREQUAL "aarch64")
else ()
endif ()

endif ()
