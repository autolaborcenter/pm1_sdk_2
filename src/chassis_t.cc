//
// Created by ydrml on 2021/3/15.
//

#include "chassis_t.hh"
#include "autocan/pm1.h"

extern "C" {
#include "../control_model/model.h"
}

#include <Windows.h>

#include <unordered_map>
#include <thread>

namespace autolabor::pm1 {
    
    using stamp_t = std::chrono::steady_clock::time_point;
    
    class chassis_t::implement_t {
        
        // config
        
        chassis_config_t _chassis_config;
        
        // serial port
        
        std::string _name;
        handle_t _serial;
        
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
        
        ~implement_t() {
            CloseHandle(_serial);
        }
        
        void write(void *buffer, size_t size) {
            #if defined __linux
            ::write(_fd, data, size);
            #elif defined WIN32
            DWORD result;
            WriteFile(_serial, buffer, size, &result, nullptr);
            #else
            #error unsupported os
            #endif
        }
    };
    
    chassis_t::chassis_t(std::string name, chassis_t::handle_t handle)
        : _implement(new implement_t(std::move(name), handle)) {
        
        // thread for sending request
        std::thread([chassis = _implement] {
            uint8_t buffer[4 * 6];
            // *reinterpret_cast<can::header_t *>(buffer) = can::pm1::node<0x3f, 0x0f>::state::tx;
            // *reinterpret_cast<can::header_t *>(buffer) = can::pm1::node<0x3f, 0x0f>::state::tx;
            // *reinterpret_cast<can::header_t *>(buffer) = can::pm1::node<0x3f, 0x0f>::state::tx;
            // *reinterpret_cast<can::header_t *>(buffer) = can::pm1::node<0x3f, 0x0f>::state::tx;
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0; chassis->active; ++i) {
                using namespace std::chrono_literals;
                constexpr static auto PERIOD = 20ms;
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
            delete chassis;
        }).detach();
    }
    
    chassis_t::~chassis_t() {
        _implement->active = false;
    }
    
}
