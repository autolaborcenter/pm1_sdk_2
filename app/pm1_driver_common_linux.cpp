#include "pm1_driver_common.h"

#include "serial_linux.h"

#include <unistd.h>// close

#include <thread>

std::unordered_map<std::string, autolabor::pm1::chassis_t>
scan_chassis(std::mutex &mutex, std::condition_variable &signal) {
    using namespace autolabor::pm1;

    std::unordered_map<std::string, chassis_t> chassis;
    for (const auto &name : list_ports()) {
        auto fd = open_serial(name);
        if (fd < 0) continue;
        auto map_iterator = chassis.try_emplace(name).first;
        std::shared_ptr<int> fd_ptr(
            new int(fd),
            [map_iterator, &chassis, &mutex, &signal](auto p) {
                close(*p);
                delete p;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    chassis.erase(map_iterator);
                }
                signal.notify_all();
            });

        std::thread([ptr = &map_iterator->second, fd_ptr] {
            uint8_t buffer[64];
            uint8_t size = 0;
            do {
                auto n = read(*fd_ptr, buffer + size, sizeof(buffer) - size);
                if (n < 0) break;
                auto buffer_ = buffer;
                auto size_ = static_cast<uint8_t>(size + n);
                ptr->communicate(buffer_, size_);
                size = buffer_ - buffer;
                if (write(*fd_ptr, buffer_, size_) != size_) break;
            } while (ptr->alive());
            ptr->close();
        }).detach();

        std::thread([ptr = &map_iterator->second, fd_ptr] {
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0; ptr->alive(); ++i) {
                auto meta = ptr->next_to_send();
                std::this_thread::sleep_until(meta.time);
                if (write(*fd_ptr, meta.msg, meta.size) <= 0) break;
            }
            ptr->close();
        }).detach();
    }

    return chassis;
}
