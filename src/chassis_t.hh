//
// Created by ydrml on 2021/3/15.
//

#ifndef PM1_SDK_2_CHASSIS_T_HH
#define PM1_SDK_2_CHASSIS_T_HH

#include <string>

namespace autolabor::pm1 {
    class chassis_t {
        using handle_t =
        #if defined __linux
        int;
        #elif defined WIN32
        void *;
        #else
        #error unsupported os
        #endif // handle_t
        
        class implement_t;
        
        implement_t *_implement;
    
    public:
        chassis_t(std::string, handle_t);
        
        ~chassis_t();
    };
}

#endif //PM1_SDK_2_CHASSIS_T_HH
