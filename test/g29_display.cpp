#include "../g29/steering_t.hh"

#include <thread>
#include <iostream>

int main() {
    float speed, rudder;
    while (true) {
        while (wait_event(speed, rudder))
            std::cout << speed << " | " << rudder << std::endl;
        std::cout << "disconnected" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
