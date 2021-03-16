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
        std::vector<uint8_t> _buffer;
        
        // chassis state
        
        union controller_t {
            uint16_t key;
            struct { uint8_t type, index; } data;
        };
        
        std::unordered_map<decltype(controller_t::key), uint8_t> _states;
        stamp_t _last_received;
        float _battery_percent;
        physical _target;
    
    public:
        bool active;
    
        implement_t(std::string &&name, chassis_t::handle_t handle)
            : _chassis_config(default_config),
              _name(std::move(name)),
              _serial(handle),
              _last_received{},
              _battery_percent{},
              _target{0, NAN},
              active(true) {}
    
        [[nodiscard]] const char *name() const {
            return _name.c_str();
        }
    
        void write(void *buffer, size_t size) {
            std::cout << "write " << size << std::endl;
            #if defined __linux
            ::write(_fd, data, size);
            #elif defined WIN32
            DWORD result;
            WriteFile(_serial, buffer, size, &result, NULL);
            #else
            #error unsupported os
            #endif
        }
    
        void read() {
            #if defined __linux
            // TODO
            #elif defined WIN32
            auto s0 = _buffer.size();
            _buffer.resize(s0 + 32);
            DWORD result;
            std::cout << "read " << _buffer.size() << std::endl;
            if (!ReadFile(_serial, _buffer.data() + s0, 32, &result, NULL)) {
                std::cout << +GetLastError() << std::endl;
                active = false;
                return;
            }
            _buffer.resize(s0 + result);
            #else
            #error unsupported os
            #endif
        
            auto slices = can::split(_buffer.data(), _buffer.data() + _buffer.size());
            for (auto i = 0; i < slices.size() - 1; ++i) {
                auto header = *reinterpret_cast<const can::header_t *>(slices[i]);
                std::cout << std::hex << +NODE_TYPE(header) << '[' << +header.data.node_index << "]: " << +header.data.msg_type << std::endl;
            }
            _buffer.erase(_buffer.begin(), _buffer.begin() + (slices.back() - _buffer.data()));
        }
    };
    
    chassis_t::chassis_t(std::string name, chassis_t::handle_t handle)
        : _implement(new implement_t(std::move(name), handle)) {
    
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
                buffer[i + 5] = can::crc_calculate(buffer + i, buffer + i + 6);
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
            #if defined __linux
            ::close(handle);
            #elif defined WIN32
            CloseHandle(handle);
            #else
            #error unsupported os
            #endif
        }).detach();
    
        // thread for receiving
        //        std::thread([chassis = _implement] {
        //            std::cout << chassis->name() << " opened." << std::endl;
        //            while (chassis->active)
        //                chassis->read();
        //            std::cout << chassis->name() << " closed." << std::endl;
        //            delete chassis;
        //        }).detach();
    }
    
    chassis_t::~chassis_t() {
        _implement->active = false;
    }
    
}
