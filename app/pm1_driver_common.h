#ifndef PM1_DRIVER_COMMON_H
#define PM1_DRIVER_COMMON_H

#include "src/chassis_t.hh"

#include <condition_variable>
#include <string>
#include <unordered_map>

std::weak_ptr<autolabor::pm1::chassis_t>
scan_chassis(
    std::mutex &,
    std::condition_variable &);

#endif// PM1_DRIVER_COMMON_H
