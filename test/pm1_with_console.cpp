#include "../app/pm1_driver_common.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>

int main() {
    std::mutex mutex;
    std::condition_variable signal;

    auto chassis = scan_chassis(mutex, signal);
    std::thread([&chassis, &mutex, &signal] {
        char line[64];
        while (std::cin.getline(line, sizeof line))
            switch (line[0]) {
                case 'N': {
                    std::cout << "N ";
                    std::unique_lock<decltype(mutex)> lock(mutex);
                    for (const auto &[name, _] : chassis) std::cout << name << ' ';
                    std::cout << std::endl;
                } break;
                case 'P': {
                    std::stringstream formatter;
                    formatter.str(line + 2);

                    std::string name;
                    float speed, rudder;

                    formatter >> name >> speed;
                    if (!formatter.fail()) {
                        std::unique_lock<decltype(mutex)> lock(mutex);
                        auto failed = true;
                        auto p = chassis.find(name);
                        if (p != chassis.end()) {
                            formatter >> rudder;
                            if ((failed = formatter.fail()) && speed == 0) {
                                rudder = NAN;
                                failed = false;
                            }
                        }
                        if (!failed) {
                            physical target{speed, rudder};
                            p->second.update(target);
                            std::cout << "P " << name << ' ' << speed << ' ' << rudder << std::endl;
                        }
                    }
                } break;
                case 'B': {
                    std::cout << "B ";
                    std::unique_lock<decltype(mutex)> lock(mutex);
                    for (const auto &[name, object] : chassis)
                        std::cout << name << ' ' << +object.battery_percent() << ' ';
                    lock.unlock();
                    std::cout << std::endl;
                } break;
            }
        {
            std::unique_lock<std::mutex> lock(mutex);
            chassis.clear();
        }
        signal.notify_all();
    }).detach();

    std::unique_lock<decltype(mutex)> lock(mutex);
    while (!chassis.empty()) signal.wait(lock);

    return 0;
}
