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

    std::thread([&mutex, &steering] {
        float speed, rudder;
        while (steering.wait_event(speed, rudder, -1)) {
            std::stringstream builder;
            builder << speed << ' ' << rudder;
            std::lock_guard<decltype(mutex)> lock(mutex);
            std::cout << builder.str() << std::endl;
        }
    }).detach();

    std::string line;
    while (std::getline(std::cin, line)) {
    }

    return 0;
}
