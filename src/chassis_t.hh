//
// Created by ydrml on 2021/3/15.
//

#ifndef PM1_SDK_2_CHASSIS_T_HH
#define PM1_SDK_2_CHASSIS_T_HH

#include <string>
#include <memory>

namespace autolabor::pm1 {
    class chassis_t {
        class implement_t;
    
        std::shared_ptr<implement_t> _implement;

    public:
        explicit chassis_t(std::string);
    
        ~chassis_t();
    
        std::pair<uint8_t, uint8_t> communicate(uint8_t *buffer, uint8_t size);
    };
}

#endif //PM1_SDK_2_CHASSIS_T_HH
