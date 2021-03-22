#ifndef PM1_DRIVER_COMMON_H
#define PM1_DRIVER_COMMON_H

#include "../src/chassis_t.hh"

#include <chrono>
#include <unordered_map>

#include <thread>
#include <condition_variable>

namespace autolabor::pm1 {
    // 应该使用一个线程循环发送的询问信息
    // 基于一个基本周期（40 ms），询问后轮角度、前轮编码器、节点状态和电池电量
    
    class loop_msg_t {
        uint8_t _bytes[4 * 6];
    
    public:
        constexpr static auto PERIOD = std::chrono::milliseconds(40);
        
        loop_msg_t();
        
        std::pair<uint8_t *, size_t> operator[](uint64_t) const;
    };
    
    std::thread launch_parser(std::mutex &, std::condition_variable &, std::unordered_map<std::string, chassis_t> &);
}

#endif // PM1_DRIVER_COMMON_H
