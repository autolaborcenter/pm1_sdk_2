#include "app/pm1_driver_common.h"
#include "app/serial_linux.h"
#include "src/servo.hpp"

#include <linux/socket.h>
#include <netinet/in.h>
#include <unistd.h>// close

#include <iostream>
#include <thread>

using namespace autolabor::pm1;

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "need a port to bind" << std::endl;
        return 1;
    }
    int port;
    try {
        port = std::stoi(argv[1]);
    } catch (...) {
        return 1;
    }

    std::mutex mutex;
    std::condition_variable signal;

    auto chassis = scan_chassis(mutex, signal);

    using clock = std::chrono::steady_clock;
    auto control_timeout = clock::now();

    servo_t servo;
    std::thread([&] {
        for (const auto &name : list_ports()) {
            servo_t temp(open_serial(name));
            if (temp) {
                servo = std::move(temp);
                std::cout << "servo name = " << name << std::endl;
                break;
            }
        }
        while (servo) {
            auto now = clock::now();
            if (now < control_timeout) {
                std::this_thread::sleep_until(control_timeout);
                continue;
            }
            auto rudder = chassis.lock()->current().rudder;
            servo(1475 - rudder / pi_f * (1475 - 501) * 2);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }).detach();

    std::thread([&, port] {
        auto udp = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in local{.sin_family = AF_INET, .sin_port = static_cast<in_port_t>(port)};
        bind(udp, reinterpret_cast<sockaddr *>(&local), sizeof(local));

        uint8_t buffer[16];
        while (true) {
            auto n = read(udp, buffer, sizeof(buffer));
            if (n == sizeof(physical)) {
                auto temp = reinterpret_cast<physical *>(buffer);
                std::cout << temp->speed << " | " << temp->rudder << std::endl;

                chassis.lock()->set_target(*temp);

                control_timeout = clock::now() + std::chrono::milliseconds(2000);
                auto dir = temp->speed < 0
                               ? 1475 + temp->rudder / pi_f * 500
                           : temp->speed > 0
                               ? 1475 - temp->rudder / pi_f * 500
                               : 1475 - temp->rudder / pi_f * (1475 - 501) * 2;
                servo(dir);
            }
        }
    }).detach();

    std::unique_lock<decltype(mutex)> lock(mutex);
    signal.wait(lock, [&chassis] { return !chassis.lock(); });

    return 0;
}
