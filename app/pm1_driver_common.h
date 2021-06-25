#ifndef PM1_DRIVER_COMMON_H
#define PM1_DRIVER_COMMON_H

#include "src/chassis_t.hh"

#include <condition_variable>
#include <string>
#include <unordered_map>

std::unordered_map<std::string, autolabor::pm1::chassis_t>
scan_chassis(std::mutex &mutex, std::condition_variable &signal);

#endif// PM1_DRIVER_COMMON_H
