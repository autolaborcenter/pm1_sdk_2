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

    // 反馈事件
    std::thread([&] {
        uint8_t level;
        float speed, rudder;
        while (steering.wait_event(level, speed, rudder, -1)) {
            predictor.set_target({speed, rudder});

            std::stringstream builder;
            builder << "T " << +level << ' ' << speed << ' ' << rudder;
            std::lock_guard<decltype(mutex)> lock(mutex);
            std::cout << builder.str() << std::endl;
        }
    }).detach();

    // 预测轨迹
    std::string line;
    while (std::getline(std::cin, line)) {
        std::stringstream builder(line);
        physical state;
        if (builder >> state.speed >> state.rudder) {
            predictor.set_current(state);
            predictor.freeze();

            autolabor::odometry_t<> pose{}, delta{};
            builder.str("");
            builder.clear();
            builder << "P 0,0,0";
            for (auto i = 0;
                 i < 500 &&
                 std::abs(pose.theta) < pi_f * 2 &&
                 std::hypotf(pose.x, pose.y) < 3 &&
                 predictor(delta);
                 ++i)
                if (std::abs(delta.s) > .05f) {
                    pose += delta.as_delta();
                    delta = {};
                    builder << ' ' << pose.x << ',' << pose.y << ',' << pose.theta;
                }

            std::lock_guard<decltype(mutex)> lock(mutex);
            std::cout << builder.str() << std::endl;
        }
    }

    return 0;
}
