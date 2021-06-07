#include "pm1_driver_common.h"
#include "serial_linux.h"
#include "servo.hpp"

#include <linux/socket.h>
#include <netinet/in.h>
#include <unistd.h>// close

#include <iostream>
#include <thread>

using namespace autolabor::pm1;

int main(int argc, char *argv[]) {
    if (argc < 2) return 1;
    int port;
    try {
        port = std::stoi(argv[1]);
    } catch (...) {
        return 1;
    }

    std::mutex mutex;
    std::condition_variable signal;

    auto chassis = scan_chassis(mutex, signal);
    servo_t servo;

    {
        std::unique_lock<decltype(mutex)> lock(mutex);
        while (chassis.size() > 1) signal.wait(lock);
        if (chassis.size() == 1) {
            for (const auto &name : list_ports()) {
                servo_t temp(open_serial(name));
                if (temp) {
                    servo = std::move(temp);
                    std::cout << "servo name = " << name << std::endl;
                    break;
                }
            }
        } else
            return 1;
    }

    using clock = std::chrono::steady_clock;
    auto control_timeout = clock::now();

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

                std::unique_lock<std::mutex> lock(mutex);
                if (chassis.size() == 1) {
                    chassis.begin()->second.set_target(*temp);
                    lock.unlock();

                    control_timeout = clock::now() + std::chrono::milliseconds(2000);
                    auto dir = temp->speed < 0
                                   ? 1475 + temp->rudder / pi_f * 500
                               : temp->speed > 0
                                   ? 1475 - temp->rudder / pi_f * 500
                                   : 1475 - temp->rudder / pi_f * (1475 - 501) * 2;
                    servo(dir);
                } else {
                    lock.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
        }
    }).detach();

    std::thread([&] {
        while (true) {
            auto now = clock::now();
            if (now < control_timeout) {
                std::this_thread::sleep_until(control_timeout);
                continue;
            }
            std::unique_lock<decltype(mutex)> lock(mutex);
            if (chassis.size() == 1) {
                servo(1475 - chassis.begin()->second.current().rudder / pi_f * (1475 - 501) * 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } else
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }).detach();

    std::unique_lock<decltype(mutex)> lock(mutex);
    while (!chassis.empty()) signal.wait(lock);

    return 0;
}
