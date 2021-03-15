//
// Created by ydrml on 2021/3/15.
//

#include "chassis_t.hh"
#include "autocan/protocol.h"

extern "C" {
#include "../control_model/model.h"
}

#include <Windows.h>

#include <memory>
#include <unordered_map>
#include <thread>

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
        // TODO send external
    }
};

chassis_t::chassis_t(std::string name, chassis_t::handle_t handle)
    : _implement(new implement_t(std::move(name), handle)) {
    
    // thread for sending request
    std::thread([chassis = _implement] {
        uint8_t buffer[4 * 6];
        for (uint64_t i = 0; chassis->active; ++i) {
            std::this_thread::sleep_for(std::chrono::microseconds(20));
            auto size = i % (1000 / 20 * 60) == 0
                        ? 24
                        : i % (1000 / 20) == 0
                          ? 18
                          : i % 2 == 0
                            ? 12
                            : 6;
            chassis->write(buffer, size);
        }
        delete chassis;
    }).detach();
}

chassis_t::~chassis_t() {
    _implement->active = false;
}
