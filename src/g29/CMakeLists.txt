cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

EXECUTE_PROCESS(COMMAND uname -m OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE architecture)
if(architecture STREQUAL "aarch64")
    add_library(g29_aarch64 SHARED steering_t.hh steering_t.cc)
else()
    add_library(g29_unix SHARED steering_t.hh steering_t.cc)
endif()
