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

    auto chassis = scan_chassis(mutex, signal);

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
                            chassis.lock()->set_target(target);
                        } catch (...) {}
                    }

                } break;
                case 'S': {
                    std::unique_lock<decltype(mutex)> lock(mutex);
                    auto current = chassis.lock()->current();
                    auto odometry = chassis.lock()->odometry();
                    std::cout << "S "
                              << +chassis.lock()->battery_percent() << ' '
                              << current.speed << ' '
                              << current.rudder << ' '
                              << odometry.s << std::endl;
                } break;
            }
        {
            std::unique_lock<std::mutex> lock(mutex);
            chassis.lock()->close();
        }
        signal.notify_all();
    }).detach();

    std::unique_lock<decltype(mutex)> lock(mutex);
    signal.wait(lock, [&chassis] { return !chassis.lock(); });

    return 0;
}
