#include "app/pm1_driver_common.h"
#include "src/g29/steering_t.hh"

#include <iostream>
#include <thread>

int main() {
    std::mutex mutex;
    std::condition_variable signal;

    auto chassis = scan_chassis(mutex, signal);
    std::thread([&chassis, &mutex] {
        physical target;
        while (true) {
            while (wait_event(target.speed, target.rudder, -1)) {
                std::cout << target.speed << " | " << target.rudder << std::endl;
                std::unique_lock<decltype(mutex)> lock(mutex);
                if (chassis.size() == 1)
                    chassis.begin()->second->set_target(target);
            }
            std::cout << "disconnected" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }).detach();

    std::unique_lock<decltype(mutex)> lock(mutex);
    while (!chassis.empty()) signal.wait(lock);

    return 0;
}
