#include "src/g29/steering_t.hh"
#include "src/predictor_t.hh"

#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

int main() {
    std::mutex mutex;
    auto steering = steering_t::scan();
    if (!steering) return 0;

    autolabor::pm1::predictor_t predictor;

    std::thread([&] {
        float speed, rudder;
        while (steering.wait_event(speed, rudder, -1)) {
            predictor.set_target({speed, rudder});

            std::stringstream builder;
            builder << speed << ' ' << rudder;
            std::lock_guard<decltype(mutex)> lock(mutex);
            std::cout << builder.str() << std::endl;
        }
    }).detach();

    std::string line;
    while (std::getline(std::cin, line)) {
        std::stringstream builder(line);
        physical state;
        if (builder >> state.speed >> state.rudder) {
            predictor.set_current(state);
            predictor.freeze();

            autolabor::odometry_t<> pose{};
            builder.str("");
            builder.clear();
            builder << "0,0,0";
            for (auto i = 0; i < 500 && predictor(pose) && std::abs(pose.theta) < pi_f / 2; ++i)
                if (i % 5 == 0) builder << ' ' << pose.x << ',' << pose.y << ',' << pose.theta;

            std::lock_guard<decltype(mutex)> lock(mutex);
            std::cout << builder.str() << std::endl;
        }
    }

    return 0;
}
