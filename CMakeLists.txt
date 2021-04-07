cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pm1_sdk_2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)

include_directories(src)

set(CONTROL_MODEL_SOURCE 
        src/control_model/pi.h

        src/control_model/model.h
        src/control_model/model.c

        src/control_model/chassis_config_t.h
        src/control_model/chassis_config_t.c

        src/control_model/motor_map.h

        src/control_model/optimization.h
        src/control_model/optimization.c)

add_library(control_model ${CONTROL_MODEL_SOURCE})

add_library(kernel
        src/autocan/protocol.h
        src/autocan/pm1.h
        src/autocan/parse.cpp

        src/chassis_t.hh
        src/chassis_t.cc

        src/chassis_model_t.hh
        src/chassis_model_t.cc)
target_link_libraries(kernel control_model)

find_package(Threads REQUIRED)
add_library(app_common app/pm1_driver_common.h app/pm1_driver_common.cpp)
target_link_libraries(app_common kernel ${CMAKE_THREAD_LIBS_INIT})

if (WIN32)

    add_executable(pm1_driver_win32 app/pm1_driver_win32.cpp)
    target_link_libraries(pm1_driver_win32 app_common)
elseif (UNIX)

    EXECUTE_PROCESS(COMMAND uname -m OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE architecture)

    add_executable(g29_test test/g29_display.cpp)
    add_executable(pm1_g29_test test/pm1_with_g29.cpp)

    if (architecture STREQUAL "aarch64")
        add_library(g29_aarch64 SHARED src/g29/steering_t.hh src/g29/steering_t.cc ${CONTROL_MODEL_SOURCE} src/odometry_t.hpp)

        add_executable(pm1_driver_aarch64 app/pm1_driver_linux.cpp)
        target_link_libraries(pm1_driver_aarch64 app_common)

        target_link_libraries(g29_test g29_aarch64)
        target_link_libraries(pm1_g29_test g29_aarch64 app_common)
    else ()
        add_library(g29_unix SHARED src/g29/steering_t.hh src/g29/steering_t.cc ${CONTROL_MODEL_SOURCE} src/odometry_t.hpp)

        add_executable(pm1_driver_unix app/pm1_driver_linux.cpp)
        target_link_libraries(pm1_driver_unix app_common)

        target_link_libraries(g29_test g29_unix)
        target_link_libraries(pm1_g29_test g29_unix app_common)
    endif ()

endif ()
