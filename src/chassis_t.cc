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
#include <algorithm>
#include <iostream>

namespace autolabor::pm1 {
    class chassis_t::implement_t {
        
        // config
        
        chassis_config_t _chassis_config;
        
        // serial port
        
        std::string _name;
        
        // chassis state
    
        union controller_t {
            uint16_t key;
            struct { uint8_t type, index; } data;
        };
    
        std::unordered_map<decltype(controller_t::key), uint8_t> _states;
        float _battery_percent;
        physical _target;

    public:
        bool active;
    
        explicit implement_t(std::string &&name)
            : _chassis_config(default_config),
              _name(std::move(name)),
              _battery_percent{},
              _target{0, NAN},
              active(true) {
        
        }
    
        std::pair<uint8_t, uint8_t> communicate(uint8_t *buffer, uint8_t size) {
            std::vector<uint8_t> reply;
            bool release = false;
        
            auto slices = can::split(buffer, buffer + size);
            auto p = slices.begin();
            auto ptr = *p;
            for (++p; p != slices.end(); ptr = *p++) {
                using namespace can::pm1;
                constexpr static auto STATE = any_node::state::rx.data.msg_type;
                constexpr static can::header_t MASK{.data{.head = 0xff, .node_type_h = 0b11, .payload = true, .node_type_l = 0b1111, .msg_type = 0xff}};
    
                auto header = reinterpret_cast<const can::header_t *>(ptr);
                auto node = controller_t{.data{.type = NODE_TYPE(header), .index = header->data.node_index}};
                if (header->data.msg_type == STATE) {
                    _states[node.key] = ptr[5];
                    release = release || ptr[5] != 1;
                } else if (!((header->key ^ any_tcu::current_position::rx.key) & MASK.key)) {
                    uint8_t data[2];
                    std::reverse_copy(ptr + 5, ptr + 7, data);
                    auto rudder = *reinterpret_cast<int16_t *>(data);
                    if (node.data.index == 0)
                        std::cout << "R0 " << rudder << std::endl;
                } else if (!((header->key ^ any_ecu::current_position::rx.key) & MASK.key)) {
                    uint8_t data[4];
                    std::reverse_copy(ptr + 5, ptr + 9, data);
                    auto pulse = *reinterpret_cast<int32_t *>(data);
                    std::cout << "P" << +header->data.node_index << ' ' << pulse << std::endl;
                } else {
                    std::cout << std::hex << "0x" << +node.data.type << '[' << +node.data.index << "]: " << +header->data.msg_type << std::endl;
                }
            }
            auto end = (size -= ptr - buffer);
            std::memcpy(buffer, ptr, size);
            if (release) {
                auto item = buffer + end;
                *reinterpret_cast<can::header_t *>(item) = can::pm1::every_node::unlock;
                item[5] = 0xff;
                item[13] = can::crc_calculate(item + 1, item + 13);
                end += 14;
            }
            return {size, end};
        }
    };
    
    chassis_t::chassis_t(std::string name)
        : _implement(std::make_shared<implement_t>(std::move(name))) {}
    
    chassis_t::~chassis_t() {
        _implement->active = false;
    }
    
    std::pair<uint8_t, uint8_t> chassis_t::communicate(uint8_t *buffer, uint8_t size) {
        return _implement->communicate(buffer, size);
    }
}
