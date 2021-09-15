#include "src/g29/steering_t.hh"
#include "src/predictor_t.hh"

#include <condition_variable>
#include <iostream>
#include <sstream>

int main() {
    auto steering = logitech::steering_t::scan();
    if (!steering) return 0;

    std::mutex mutex_cout, mutex_prodict;
    std::condition_variable signal;

    // 反馈事件
    uint8_t level;
    float speed, rudder;
    auto stop = true;
    while (steering.wait_event(level, speed, rudder, -1)) {
        auto now = -.01 < speed && speed < .01;
        if (std::exchange(stop, now) && !now) signal.notify_one();

        std::stringstream builder;
        builder << +level << ' ' << speed << ' ' << rudder;
        std::lock_guard<decltype(mutex_cout)> lock(mutex_cout);
        std::cout << builder.str() << std::endl;
    }

    return 0;
}
