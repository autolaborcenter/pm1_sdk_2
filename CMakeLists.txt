cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pm1_sdk_2 LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(control_model
        src/control_model/pi.h

        src/control_model/model.h
        src/control_model/model.c

        src/control_model/chassis_config_t.h
        src/control_model/chassis_config_t.c

        src/control_model/motor_map.h

        src/control_model/optimization.h
        src/control_model/optimization.c)

add_library(chassis_model
        src/chassis_model_t.hh
        src/chassis_model_t.cc

        src/predictor_t.hh
        src/predictor_t.cc)
target_link_libraries(chassis_model control_model)

add_library(pm1
        src/autocan/protocol.h
        src/autocan/pm1.h
        src/autocan/parse.cpp

        src/chassis_t.hh
        src/chassis_t.cc)
target_link_libraries(pm1 chassis_model)

if (UNIX)

find_package(Threads REQUIRED)

add_library(pm1_linux
        app/pm1_driver_common.h
        app/pm1_driver_common_linux.cpp
        app/serial_linux.h
        app/serial_linux.cpp)
target_link_libraries(pm1_linux pm1 pthread)

add_executable(path_predictor_unix app/path_predictor.cpp)
target_link_libraries(path_predictor_unix chassis_model)

# 架构相关编译

EXECUTE_PROCESS(COMMAND uname -m OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE architecture)
if (architecture STREQUAL "aarch64")

add_executable(pm1_console_aarch64 app/pm1_with_console.cpp)
target_link_libraries(pm1_console_aarch64 pm1_linux)

else ()

# PM1

add_executable(pm1_console_unix app/pm1_with_console.cpp)
target_link_libraries(pm1_console_unix pm1_linux)

# 罗技 g29

add_library(g29 src/g29/steering_t.hh src/g29/steering_t.cc)
target_link_libraries(g29 chassis_model)

add_executable(g29_console_unix app/g29_console.cpp)
target_link_libraries(g29_console_unix g29 pthread)

# test

add_executable(pm1_udp test/pm1_with_udp.cpp)
target_link_libraries(pm1_udp pm1_linux)

add_executable(g29_udp test/g29_with_udp.cpp)
target_link_libraries(g29_udp g29)

add_executable(servo_test test/servo_test.cpp
        app/serial_linux.h
        app/serial_linux.cpp)

endif ()

else ()

add_executable(path_predictor_win32 app/path_predictor.cpp)
target_link_libraries(path_predictor_win32 chassis_model)

endif ()
