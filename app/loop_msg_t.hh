//
// Created by ydrml on 2021/3/18.
//

#ifndef PM1_SDK_2_LOOP_MSG_T_HH
#define PM1_SDK_2_LOOP_MSG_T_HH

#include <vector>
#include <chrono>

namespace autolabor::pm1 {
    class loop_msg_t {
        uint8_t _bytes[4 * 6];
    
    public:
        constexpr static auto PERIOD = std::chrono::milliseconds(40);
        
        loop_msg_t();
        
        std::pair<uint8_t *, size_t> operator[](uint64_t) const;
    };
}

#endif //PM1_SDK_2_LOOP_MSG_T_HH
