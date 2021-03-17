//
// Created by ydrml on 2021/3/15.
//

#include "chassis_t.hh"
#include "autocan/pm1.h"

extern "C" {
#include "../control_model/model.h"
}

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
    
        std::pair<uint8_t, uint8_t> communicate(uint8_t *buffer, uint8_t size) {
            auto slices = can::split(buffer, buffer + size);
            auto p = slices.begin();
            auto ptr = *p;
            for (++p; p != slices.end(); ptr = *p++) {
                auto header = reinterpret_cast<const can::header_t *>(ptr);
                std::cout << std::hex << +NODE_TYPE(*header) << '[' << +header->data.node_index << "]: " << +header->data.msg_type << std::endl;
            }
            std::memcpy(buffer, ptr, size -= ptr - buffer);
            std::cout << "size = " << +size << std::endl;
            return {size, size};
        }
    };
    
    chassis_t::chassis_t(std::string name, chassis_t::handle_t handle)
        : _implement(std::make_shared<implement_t>(std::move(name), handle)) {}
    
    chassis_t::~chassis_t() {
        _implement->active = false;
    }
    
    std::pair<uint8_t, uint8_t> chassis_t::communicate(uint8_t *buffer, uint8_t size) {
        return _implement->communicate(buffer, size);
    }
}
