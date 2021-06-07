#include "../app/serial_linux.h"
#include "servo.hpp"

#include <iostream>

int main() {
    servo_t servo;
    for (const auto &name : list_ports()) {
        servo_t temp(open_serial(name));
        if (temp) {
            servo = std::move(temp);
            std::cout << "servo name = " << name << std::endl;
            break;
        }
    }
    std::string line;
    if (servo)
        while (std::getline(std::cin, line))
            try {
                servo(std::stoi(line));
            } catch (...) {
            }

    return 0;
}
