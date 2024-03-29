﻿#include "app/pm1_driver_common.h"
#include "src/predictor_t.hh"

#include <algorithm>
#include <iostream>
#include <ranges>
#include <sstream>
#include <thread>

bool parse(const char *text, physical &p) {
    std::stringstream formatter(text);
    float speed;
    std::string rudder;

    if (formatter >> speed >> rudder) {
        std::ranges::copy(rudder | std::views::transform([](auto c) { return std::tolower(c); }), rudder.begin());
        try {
            p = {speed, rudder == "nan" ? NAN : std::stof(rudder)};
            return true;
        } catch (...) {}
    }
    return false;
}

int main() {
    using namespace autolabor::pm1;

    std::mutex mutex;
    std::condition_variable signal;

    auto chassis = scan_chassis(mutex, signal);
    predictor_t predictor;

    std::thread([&, chassis] {
        char line[64];
        while (std::cin.getline(line, sizeof line))
            switch (line[0]) {
                // [S]tate
                case 'S': {
                    std::unique_lock<decltype(mutex)> lock(mutex);
                    auto ptr = chassis.lock();
                    auto current = ptr->current();
                    auto odometry = ptr->odometry();
                    std::cout
                        << "S "
                        << +ptr->battery_percent() << ' '
                        << current.speed << ' '
                        << current.rudder << ' '
                        << odometry.s << ' '
                        << odometry.a << ' '
                        << odometry.x << ' '
                        << odometry.y << ' '
                        << odometry.theta << std::endl;
                } break;
                // [T]arget
                case 'T': {
                    physical target, current;
                    if (parse(line + 2, target)) {
                        {
                            std::unique_lock<decltype(mutex)> lock(mutex);
                            current = chassis.lock()->current();
                        }
                        predictor.set_current(current);
                        predictor.set_target(target);
                        predictor.freeze();
                        // 最大速度 1m/s，最大加速度 0.8m/s
                        // 减速时间 1.25s
                        // 维持超时 200ms
                        // 路径 := 间隔 0.1s 的 20 个点
                        // 0.1s x 20 = 2s > 1.95s = 1.25s + 200ms + 500ms
                        // [P]ath: P speed|rudder {x,y,theta}...
                        std::cout << "P " << target.speed << '|' << target.rudder;
                        autolabor::odometry_t<> pose{};
                        for (auto i = 0; i < 20; ++i) {
                            for (auto j = 0; j < 5; ++j)
                                if (!predictor(pose)) {
                                    i = 20;
                                    break;
                                }
                            std::cout << ' ' << static_cast<int>(pose.x * 1000)
                                      << ',' << static_cast<int>(pose.y * 1000)
                                      << ',' << pose.theta;
                        }
                        std::cout << std::endl;
                    }
                } break;
                // [P]hysical
                case 'P': {
                    physical target;
                    if (parse(line + 2, target)) {
                        std::unique_lock<decltype(mutex)> lock(mutex);
                        chassis.lock()->set_target(target);
                    }
                } break;
            }
        std::cerr << "cin closed!" << std::endl;
        {
            std::unique_lock<std::mutex> lock(mutex);
            chassis.lock()->close();
        }
        signal.notify_all();
    }).detach();

    std::unique_lock<decltype(mutex)> lock(mutex);
    signal.wait(lock, [&chassis] { return !chassis.lock(); });
    std::cerr << "chassis closed!" << std::endl;

    return 0;
}
