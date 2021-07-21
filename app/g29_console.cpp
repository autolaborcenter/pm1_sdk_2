#include "src/g29/steering_t.hh"
#include "src/predictor_t.hh"

#include <condition_variable>
#include <iostream>
#include <sstream>
#include <thread>

int main() {
    auto steering = steering_t::scan();
    if (!steering) return 0;

    autolabor::pm1::predictor_t predictor;
    std::mutex mutex_cout, mutex_prodict;
    std::condition_variable signal;

    // 反馈事件
    std::thread([&] {
        uint8_t level;
        float speed, rudder;
        auto stop = true;
        while (steering.wait_event(level, speed, rudder, -1)) {
            predictor.set_target({speed, rudder});
            auto now = -.01 < speed && speed < .01;
            if (std::exchange(stop, now) && !now) signal.notify_one();

            std::stringstream builder;
            builder << "T " << +level << ' ' << speed << ' ' << rudder;
            std::lock_guard<decltype(mutex_cout)> lock(mutex_cout);
            std::cout << builder.str() << std::endl;
        }
    }).detach();

    // 输出预测轨迹
    std::thread([&] {
        while (true) {
            std::unique_lock<decltype(mutex_prodict)> lock(mutex_prodict);
            signal.wait(lock);

            predictor.freeze();

            std::stringstream builder;
            autolabor::odometry_t<> pose{}, delta{};
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

            std::lock_guard<decltype(mutex_cout)> lock(mutex_cout);
            std::cout << builder.str() << std::endl;
        }
    }).detach();

    // 输入机器人状态
    std::string line;
    physical state;
    while (std::getline(std::cin, line)) {
        std::stringstream builder(line);
        if (builder >> state.speed >> state.rudder) {
            predictor.set_current(state);
            signal.notify_one();
        }
    }

    return 0;
}
