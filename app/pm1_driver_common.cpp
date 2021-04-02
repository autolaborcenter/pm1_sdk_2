#include "pm1_driver_common.h"

#include "autocan/pm1.h"
#include "chassis_model_t.hh"

#include <cmath>
#include <cstring>
#include <iostream>

namespace autolabor::pm1 {
    loop_msg_t::loop_msg_t() : _bytes{} {
#define SET_HEADER(N) *reinterpret_cast<can::header_t *>(_bytes + (N)) = can::pm1
        SET_HEADER(0)::every_tcu::current_position::tx;
        SET_HEADER(6)::every_ecu::current_position::tx;
        SET_HEADER(12)::every_node::state::tx;
        SET_HEADER(18)::every_vcu::battery_percent::tx;
#undef SET_HEADER
        for (auto i = 0; i < sizeof(_bytes); i += 6)
            _bytes[i + 5] = can::crc_calculate(_bytes + i + 1, _bytes + i + 5);
    }

    std::pair<uint8_t *, size_t> loop_msg_t::operator[](uint64_t i) const {
        using namespace std::chrono_literals;
        constexpr static uint32_t
            N0 = 10s / chassis_model_t::CONTROL_PERIOD,
            N1 = 400ms / chassis_model_t::CONTROL_PERIOD,
            N2 = 2;
        auto size = i % N0 == 0
                        ? 24
                    : i % N1 == 0
                        ? 18
                    : i % N2 == 0
                        ? 12
                        : 6;
        auto buffer = new uint8_t[size];
        std::memcpy(buffer, _bytes, size);
        return {buffer, size};
    }


    std::thread launch_parser(
        std::mutex &mutex,
        std::condition_variable &signal,
        std::unordered_map<std::string, chassis_t> &chassis) {
        return std::thread([&mutex, &signal, &chassis] {
            std::stringstream formatter;
            char line[64];
            while (std::cin.getline(line, sizeof line)) {
                switch (line[0]) {
                    case 'N': {
                        std::cout << "N ";
                        std::unique_lock<std::mutex> lock(mutex);
                        for (const auto &[name, _] : chassis)
                            std::cout << name << ' ';
                        lock.unlock();
                        std::cout << std::endl;
                    } break;
                    case 'V': {
                        std::string name;
                        float v, w;
                        formatter.str(line + 2);
                        formatter >> name >> v >> w;
                        if (!formatter.fail()) {
                            std::unique_lock<std::mutex> lock(mutex);
                            auto p = chassis.find(name);
                            if (p != chassis.end())
                                p->second.set_velocity(v, w);
                        }
                    } break;
                    case 'P': {
                        std::string name;
                        float speed, rudder;
                        formatter.str(line + 2);
                        formatter >> name >> speed >> rudder;
                        if (!formatter.fail()) {
                            std::unique_lock<std::mutex> lock(mutex);
                            auto p = chassis.find(name);
                            if (p != chassis.end())
                                p->second.set_physical(speed, rudder);
                        }
                    } break;
                    case 'B': {
                        std::cout << "B ";
                        std::unique_lock<std::mutex> lock(mutex);
                        for (const auto &[name, object] : chassis)
                            std::cout << name << ' ' << +object.battery_percent() << ' ';
                        lock.unlock();
                        std::cout << std::endl;
                    } break;
                }
            }
            {
                std::unique_lock<std::mutex> lock(mutex);
                chassis.clear();
            }
            signal.notify_all();
        });
    }
}// namespace autolabor::pm1
