#include "g29/steering_t.hh"

#include <iostream>
#include <thread>

int main() {
    float speed, rudder;
    while (true) {
        while (wait_event(speed, rudder, -1))
            std::cout << speed << " | " << rudder << std::endl;
        std::cout << "disconnected" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
