cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pm1_sdk_2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)

add_subdirectory(control_model)

find_package(Threads REQUIRED)

add_library(kernel
        src/autocan/protocol.h
        src/autocan/pm1.h
        src/autocan/parse.cpp

        src/chassis_t.hh
        src/chassis_t.cc
        )
target_link_libraries(kernel control_model)

if (WIN32)
    add_executable(pm1_driver_win32 app/pm1_driver_win32.cpp
            app/pm1_driver_common.h
            app/pm1_driver_common.cpp)
    target_link_libraries(pm1_driver_win32 kernel)
elseif (UNIX)
    find_package(Threads)
    add_executable(pm1_driver_linux app/pm1_driver_linux.cpp
            app/pm1_driver_common.h
            app/pm1_driver_common.cpp)
    target_link_libraries(pm1_driver_linux kernel ${CMAKE_THREAD_LIBS_INIT})
endif ()
