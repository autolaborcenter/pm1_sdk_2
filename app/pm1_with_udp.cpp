#include "chassis_model_t.hh"
#include "pm1_driver_common.h"
#include "servo.hpp"

#include <fcntl.h>// open
#include <linux/socket.h>
#include <netinet/in.h>
#include <termios.h>// config
#include <unistd.h> // close

#include <filesystem>
#include <iostream>
#include <vector>

using namespace autolabor::pm1;

int main() {
    std::vector<std::string> ports;
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/tty"))
        if (entry.is_directory()) {
            auto name = entry.path().filename().string();
            if (name.starts_with("ttyUSB") || name.starts_with("ttyACM"))
                if (name != "ttyACM0")
                    ports.emplace_back(std::move(name));
        }

    servo_t servo;
    std::unordered_map<std::string, chassis_t> chassis;
    std::mutex mutex;
    std::condition_variable signal;
    for (const auto &name : ports) {
        std::filesystem::path path("/dev");
        auto fd = open(path.append(name).c_str(), O_RDWR);
        if (fd < 0) continue;

        // @see https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
        termios tty{};
        cfsetspeed(&tty, B115200);
        tty.c_cflag |= CS8;           // 8 bits per byte
        tty.c_cflag |= CREAD | CLOCAL;// Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_cc[VTIME] = 5;// Wait for up to 500ms, returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(fd, TCSAFLUSH, &tty)) {
            close(fd);
            continue;
        }

        using HANDLE = decltype(fd);
        auto map_iterator = chassis.try_emplace(name).first;
        std::shared_ptr<HANDLE> fd_ptr(
            new HANDLE(fd),
            [&, map_iterator](auto p) {
                servo_t temp(*p);
                if (temp) {
                    servo = std::move(temp);
                } else
                    close(*p);
                delete p;
                std::unique_lock<std::mutex> lock(mutex);
                chassis.erase(map_iterator);
                if (chassis.empty()) {
                    lock.unlock();
                    signal.notify_all();
                }
            });

        std::thread([ptr = &map_iterator->second, fd_ptr] {
            uint8_t buffer[64];
            uint8_t size = 0;
            do {
                auto n = read(*fd_ptr, buffer + size, sizeof(buffer) - size);
                if (n <= 0)
                    break;
                auto buffer_ = buffer;
                auto size_ = static_cast<uint8_t>(size + n);
                ptr->communicate(buffer_, size_);
                size = buffer_ - buffer;
                if (size_ && write(*fd_ptr, buffer_, size_) <= 0)
                    break;
            } while (ptr->alive());
            ptr->close();
        }).detach();

        std::thread([ptr = &map_iterator->second, fd_ptr] {
            auto msg = loop_msg_t();
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0; ptr->alive(); ++i) {
                auto [buffer, size] = msg[i];
                if (write(*fd_ptr, buffer, size) <= 0)
                    break;
                delete[] buffer;
                std::this_thread::sleep_until(t0 += chassis_model_t::CONTROL_PERIOD);
            }
            ptr->close();
        }).detach();
    }

    using clock = std::chrono::steady_clock;
    using stamp_t = clock::time_point;

    stamp_t control_timeout = clock::now();

    std::thread([&] {
        auto udp = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in port{.sin_family = AF_INET, .sin_port = 33333};
        bind(udp, reinterpret_cast<sockaddr *>(&port), sizeof(port));

        uint8_t buffer[16];
        while (true) {
            auto n = read(udp, buffer, sizeof(buffer));
            if (n == sizeof(physical)) {
                std::unique_lock<std::mutex> lock(mutex);
                if (chassis.size() == 1) {
                    auto temp = reinterpret_cast<physical *>(buffer);
                    if (temp->speed != 0) {
                        control_timeout = clock::now() + std::chrono::milliseconds(1500);
                        auto dir = 1450 + (temp->speed > 0 ? -temp->rudder : +temp->rudder) / pi_f * 500;
                        servo(dir);
                        std::cout << temp->speed << " | " << temp->rudder << " | " << dir << std::endl;
                    }
                    chassis.begin()->second.set_physical(temp->speed, temp->rudder);
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
            if (chassis.size() == 1) {
                float _, rudder;
                chassis.begin()->second.target(_, rudder);
                servo(1450 - rudder / pi_f * 1949);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } else
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }).detach();

    std::unique_lock<std::mutex> lock(mutex);
    while (!chassis.empty()) signal.wait(lock);

    return 0;
}
