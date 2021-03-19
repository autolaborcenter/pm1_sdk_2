//
// Created by ydrml on 2021/3/15.
//

#ifndef PM1_SDK_2_CHASSIS_T_HH
#define PM1_SDK_2_CHASSIS_T_HH

#include <string>

namespace autolabor::pm1 {
    class chassis_t {
        class implement_t;
    
        implement_t *_implement;

    public:
        chassis_t();
    
        ~chassis_t();
    
        std::pair<uint8_t, uint8_t> communicate(uint8_t *buffer, uint8_t size);
    
        [[nodiscard]] bool alive() const;
    
        void close();
    
        void set_velocity(float v, float w);
    
        void set_physical(float speed, float rudder);
    };
}

#endif //PM1_SDK_2_CHASSIS_T_HH
