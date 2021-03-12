cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pm1_sdk_2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Debug")
find_package(Threads REQUIRED)

add_library(kernel
	src/serial_port/serial_port_t.hh
	src/serial_port/serial_port_t_linux.cc
	src/serial_port/serial_port_t_win32.cc
)

if(WIN32)
	add_executable(pm1_driver_win32 "app/pm1_driver_win32.cpp")
	target_link_libraries(pm1_driver_win32 kernel)
elseif(UNIX)
	add_executable(pm1_driver_linux "app/pm1_driver_linux.cpp")
	target_link_libraries(pm1_driver_linux kernel)
endif()
