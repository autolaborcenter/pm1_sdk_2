#include "app/pm1_driver_common.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>

int main() {
    using namespace autolabor::pm1;

    std::mutex mutex;
    std::condition_variable signal;

    std::shared_ptr<chassis_t> chassis;
    {
        auto candidates = scan_chassis(mutex, signal);
        std::unique_lock<decltype(mutex)> lock(mutex);
        signal.wait(lock, [&candidates] { return candidates.size() <= 1; });
        if (candidates.empty()) return 1;
        std::cout << "N " << candidates.begin()->first << std::endl;
        chassis = std::move(candidates.begin()->second);
    }

    std::thread([chassis, &mutex, &signal] {
        char line[64];
        while (std::cin.getline(line, sizeof line))
            switch (line[0]) {
                case 'P': {
                    std::stringstream formatter;
                    formatter.str(line + 2);

                    std::string speed, rudder;

                    if (formatter >> speed >> rudder) {
                        std::transform(rudder.begin(), rudder.end(), rudder.begin(),
                                       [](auto c) { return std::tolower(c); });
                        try {
                            physical target{std::stof(speed), rudder == "nan" ? NAN : std::stof(rudder)};
                            std::cout << "P " << target.speed << ' ' << target.rudder << std::endl;
                            std::unique_lock<decltype(mutex)> lock(mutex);
                            chassis->set_target(target);
                        } catch (...) {}
                    }

                } break;
                case 'B': {
                    std::unique_lock<decltype(mutex)> lock(mutex);
                    std::cout << "B " << +chassis->battery_percent() << std::endl;
                } break;
                case 'S': {
                    std::unique_lock<decltype(mutex)> lock(mutex);
                    auto current = chassis->current();
                    auto odometry = chassis->odometry();
                    std::cout << "S "
                              << current.speed << ' '
                              << current.rudder << ' '
                              << odometry.x << ' '
                              << odometry.y << ' '
                              << odometry.theta << std::endl;
                } break;
            }
        {
            std::unique_lock<std::mutex> lock(mutex);
            chassis->close();
        }
        signal.notify_all();
    }).detach();

    std::unique_lock<decltype(mutex)> lock(mutex);
    signal.wait(lock, [&chassis] { return !chassis->alive(); });

    return 0;
}
