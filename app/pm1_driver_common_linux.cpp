#include "pm1_driver_common.h"

#include "serial_linux.h"

#include <unistd.h>// close

#include <iostream>
#include <thread>

std::weak_ptr<autolabor::pm1::chassis_t>
scan_chassis(std::mutex &mutex, std::condition_variable &signal) {
    using namespace autolabor::pm1;

    std::unordered_map<std::string, std::weak_ptr<chassis_t>> candidates;
    for (const auto &name : list_ports()) {
        auto fd = open_serial(name);
        if (fd < 0) continue;
        auto chassis = std::shared_ptr<chassis_t>(
            new chassis_t(),
            [name, fd, &candidates, &mutex, &signal](auto ptr) {
                close(fd);
                delete ptr;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    candidates.erase(name);
                }
                signal.notify_all();
            });
        candidates[name] = chassis;

        std::thread([name, chassis, fd, &signal] {
            uint8_t buffer[64];
            uint8_t size = 0;
            do {
                auto n = read(fd, buffer + size, sizeof(buffer) - size);
                if (n == 0)
                    std::cerr << "serial timeout!" << std::endl;
                else if (n < 0) {
                    std::cerr << "chassis closed by reader[" << n << "]!" << std::endl;
                    break;
                }
                auto buffer_ = buffer;
                auto size_ = static_cast<uint8_t>(size + n);
                chassis->communicate(buffer_, size_);
                size = buffer_ - buffer;
                if (write(fd, buffer_, size_) != size_) break;
            } while (chassis->alive());
            chassis->close();
            signal.notify_all();
        }).detach();

        std::thread([name, chassis, fd, &signal] {
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0; chassis->alive(); ++i) {
                auto meta = chassis->next_to_send();
                std::this_thread::sleep_until(meta.time);
                if (write(fd, meta.msg, meta.size) <= 0) {
                    std::cerr << "chassis closed by writer!" << std::endl;
                    break;
                }
            }
            chassis->close();
            signal.notify_all();
        }).detach();
    }

    std::unique_lock<std::mutex> lock(mutex);
    signal.wait(lock, [&candidates] { return candidates.size() <= 1; });
    if (candidates.empty()) return {};
    return candidates.begin()->second;
}
