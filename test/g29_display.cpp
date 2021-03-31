#include "../g29/steering_t.hh"

#include <iostream>

int main() {
    float speed, rudder;
    while (wait_event(speed, rudder))
        std::cout << speed << " | " << rudder << std::endl;

    return 0;
}
