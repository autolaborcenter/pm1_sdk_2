#include "pm1_driver_common.h"

#include "autocan/pm1.h"
#include "chassis_model_t.hh"

#include <cstring>
#include <cmath>
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
        std::unordered_map<std::string, chassis_t> &chassis
    ) {
        return std::thread([&mutex, &signal, &chassis] {
            using namespace std::chrono;
            using namespace std::chrono_literals;
            
            std::string target, temp;
            auto p0 = NAN;
            steady_clock::time_point t0;
            
            enum class STATE { IDLE, V, P } state = STATE::IDLE;
            
            while (std::cin >> temp) {
                if (state != STATE::IDLE && steady_clock::now() > t0 + 20ms)
                    state = STATE::IDLE;
                switch (state) {
                    case STATE::IDLE:
                        if (temp.size() == 1)
                            switch (temp[0]) {
                                case 'N': {
                                    std::unique_lock<std::mutex> lock(mutex);
                                    std::cout << "N ";
                                    for (const auto &[name, _] : chassis)
                                        std::cout << name << ' ';
                                    std::cout << std::endl;
                                }
                                    break;
                                case 'V':
                                    t0 = steady_clock::now();
                                    state = STATE::V;
                                    break;
                                case 'P':
                                    t0 = steady_clock::now();
                                    state = STATE::P;
                                    break;
                                case 'B': {
                                    std::unique_lock<std::mutex> lock(mutex);
                                    std::cout << "B ";
                                    for (const auto &[name, object] : chassis)
                                        std::cout << name << ' ' << +object.battery_percent() << ' ';
                                    std::cout << std::endl;
                                }
                                    break;
                            }
                        break;
                    case STATE::V: { // velocity
                        if (target.empty()) {
                            target = std::move(temp);
                            p0 = NAN;
                            break;
                        }
                        float value;
                        try {
                            value = std::stof(temp);
                        }
                        catch (...) {
                            state = STATE::IDLE;
                            break;
                        }
                        if (std::isinf(value)) {
                            state = STATE::IDLE;
                            break;
                        }
                        if (std::isnan(p0))
                            p0 = value;
                        else {
                            std::unique_lock<std::mutex> lock(mutex);
                            auto p = chassis.find(target);
                            if (p != chassis.end())
                                p->second.set_velocity(p0, value);
                            lock.unlock();
                            target.clear();
                            state = STATE::IDLE;
                        }
                        break;
                    }
                    case STATE::P: { // physical
                        if (target.empty()) {
                            target = std::move(temp);
                            p0 = NAN;
                            break;
                        }
                        float value;
                        try {
                            value = std::stof(temp);
                        }
                        catch (...) {
                            state = STATE::IDLE;
                            break;
                        }
                        if (std::isinf(value)) {
                            state = STATE::IDLE;
                            break;
                        }
                        if (std::isnan(p0))
                            p0 = value;
                        else {
                            std::unique_lock<std::mutex> lock(mutex);
                            auto p = chassis.find(target);
                            if (p != chassis.end())
                                p->second.set_physical(p0, value);
                            lock.unlock();
                            target.clear();
                            state = STATE::IDLE;
                        }
                        break;
                    }
                }
            }
            {
                std::unique_lock<std::mutex> lock(mutex);
                chassis.clear();
            }
            signal.notify_all();
        });
    }
}
