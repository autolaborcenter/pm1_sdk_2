//
// Created by ydrml on 2021/3/15.
//

#include "chassis_t.hh"
#include "autocan/pm1.h"

extern "C" {
#include "../control_model/model.h"
}

#include <Windows.h>

#include <vector>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <iostream>

namespace autolabor::pm1 {
    
    using stamp_t = std::chrono::steady_clock::time_point;
    
    class chassis_t::implement_t {
        
        // config
        
        chassis_config_t _chassis_config;
        
        // serial port
        
        std::string _name;
        handle_t _serial;
        uint8_t _buffer[64];
        
        // chassis state
    
        union controller_t {
            uint16_t key;
            struct { uint8_t type, index; } data;
        };
    
        std::unordered_map<decltype(controller_t::key), uint8_t> _states;
        stamp_t _last_received;
        float _battery_percent;
        physical _target;
    
        static void WINAPI write_callback(
            DWORD error_code,
            DWORD actual,
            LPOVERLAPPED overlapped
        ) {
            if (error_code)
                std::cerr << "write error: " << error_code << std::endl;
            delete[] reinterpret_cast<uint8_t *>(overlapped->hEvent);
            delete overlapped;
        }
    
        static void WINAPI read_callback(
            DWORD error_code,
            DWORD actual,
            LPOVERLAPPED overlapped
        ) {
            auto ptr = reinterpret_cast<chassis_t::implement_t *>(overlapped->hEvent);
            if (error_code || !ptr->active) {
                std::cerr << "read error: " << error_code << std::endl;
                delete overlapped;
                return;
            }
            auto buffer = ptr->_buffer;
            buffer[0] += actual;
            auto slices = can::split(buffer + 1, buffer + 1 + buffer[0]);
            for (auto i = 0; i < slices.size() - 1; ++i) {
                auto header = *reinterpret_cast<const can::header_t *>(slices[i]);
            }
            auto begin = slices.back();
            auto end = buffer + buffer[0] + 1;
            std::memcpy(buffer + 1, begin, end - begin);
            buffer[0] = end - begin;
            ptr->read(overlapped);
        }

    public:
        bool active;
    
        implement_t(std::string &&name, chassis_t::handle_t handle)
            : _chassis_config(default_config),
              _name(std::move(name)),
              _serial(handle),
              _buffer{},
              _last_received{},
              _battery_percent{},
              _target{0, NAN},
              active(true) {
        
        }
    
        [[nodiscard]] const char *name() const {
            return _name.c_str();
        }
    
        void write(void *buffer, size_t size) {
            #if defined __linux
            ::write(_fd, data, size);
            #elif defined WIN32
            auto overlapped = new OVERLAPPED{.hEvent = new uint8_t[size]};
            std::memcpy(overlapped->hEvent, buffer, size);
            WriteFileEx(_serial, overlapped->hEvent, size, overlapped, &write_callback);
            SleepEx(INFINITE, true);
            #else
            #error unsupported os
            #endif
        }
    
        void read(LPOVERLAPPED overlapped) {
            ReadFileEx(_serial, _buffer + _buffer[0] + 1, sizeof(_buffer) - _buffer[0] - 1, overlapped, &read_callback);
            SleepEx(INFINITE, true);
        }
    };
    
    chassis_t::chassis_t(std::string name, chassis_t::handle_t handle)
        : _implement(std::make_shared<implement_t>(std::move(name), handle)) {
        
        // thread for sending request
        std::thread([chassis = _implement, handle] {
            uint8_t buffer[4 * 6]{};
            #define SET_HEADER(N) *reinterpret_cast<can::header_t *>(buffer + N) = can::pm1
            SET_HEADER(0)::every_tcu::current_position::tx;
            SET_HEADER(6)::every_ecu::current_position::tx;
            SET_HEADER(12)::every_node::state::tx;
            SET_HEADER(18)::every_vcu::battery_percent::tx;
            #undef SET_HEADER
            for (auto i = 0; i < sizeof(buffer); i += 6)
                buffer[i + 5] = can::crc_calculate(buffer + i + 1, buffer + i + 5);
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0; chassis->active; ++i) {
                using namespace std::chrono_literals;
                constexpr static auto PERIOD = 40ms;
                constexpr static uint32_t
                    N0 = 1min / PERIOD,
                    N1 = 1s / PERIOD,
                    N2 = 2;
                auto size = i % N0 == 0
                            ? 24
                            : i % N1 == 0
                              ? 18
                              : i % N2 == 0
                                ? 12
                                : 6;
                chassis->write(buffer, size);
                t0 += PERIOD;
                std::this_thread::sleep_until(t0);
            }
        }).detach();
        
        std::thread([chassis = _implement] {
            chassis->read(new OVERLAPPED{.hEvent = chassis.get()});
        }).detach();
    }
    
    chassis_t::~chassis_t() {
        _implement->active = false;
    }
    
}
