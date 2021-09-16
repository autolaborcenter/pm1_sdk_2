#include "src/g29/steering_t.hh"

#include <iostream>

int main() {
    auto steering = logitech::steering_t::scan();

    // 反馈事件
    int8_t level;
    float speed, rudder;
    while (steering.wait_event(level, speed, rudder, -1))
        std::cout << +level << ' ' << speed << ' ' << rudder << std::endl;

    return 0;
}
