cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pm1_sdk_2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)

include_directories(src)
add_subdirectory(src/control_model)

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
    add_subdirectory(g29)

    find_package(Threads)
    EXECUTE_PROCESS(COMMAND uname -m OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE architecture)
    add_executable(g29_test test/g29_display.cpp)
    add_executable(pm1_g29_test test/pm1_with_g29.cpp)
    if (architecture STREQUAL "aarch64")
        add_executable(pm1_driver_aarch64 app/pm1_driver_linux.cpp)
        target_link_libraries(pm1_driver_aarch64 app_common)

        target_link_libraries(g29_test g29_aarch64)
        target_link_libraries(pm1_g29_test g29_aarch64 app_common)
    else ()
        add_executable(pm1_driver_unix app/pm1_driver_linux.cpp)
        target_link_libraries(pm1_driver_unix app_common)

        target_link_libraries(g29_test g29_unix)
        target_link_libraries(pm1_g29_test g29_unix app_common)
    endif ()
endif ()
