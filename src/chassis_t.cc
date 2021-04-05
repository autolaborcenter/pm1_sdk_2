//
// Created by ydrml on 2021/3/15.
//

#include "chassis_t.hh"

#include "autocan/pm1.h"
#include "chassis_model_t.hh"

extern "C" {
#include "control_model/motor_map.h"
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace autolabor::pm1 {
    class chassis_t::implement_t {
        using clock = std::chrono::steady_clock;

        union controller_t {
            uint16_t key;
            struct {
                uint8_t type, index;
            } data;
        };

        std::unordered_map<decltype(controller_t::key), uint8_t> _states;
        uint8_t _battery_percent;

        clock::time_point _target_set, _rudder_received;
        float _internal_speed;
        physical _target;
        bool _alive;

        template<class t>
        static uint8_t *to_stream(uint8_t *buffer, t value) {
            std::reverse_copy(reinterpret_cast<uint8_t *>(&value), reinterpret_cast<uint8_t *>(&value + 1), buffer + 5);
            buffer[13] = can::crc_calculate(buffer + 1, buffer + 13);
            return buffer + 14;
        }

    public:
        chassis_model_t model;

        implement_t()
            : _battery_percent(0),
              _target_set(clock::time_point::min()),
              _rudder_received(clock::now()),
              _internal_speed(0),
              _target(physical_zero),
              _alive(true) {}

        [[nodiscard]] bool alive() const {
            return _alive && clock::now() < _rudder_received + std::chrono::milliseconds(200);
        }

        [[nodiscard]] uint8_t battery_percent() const {
            return _battery_percent;
        }

        void close() {
            _alive = false;
        }

        void set_target(physical target) {
            _target_set = clock::now();
            _target = target;
        }

        void communicate(uint8_t *&buffer, uint8_t &size) {
            auto release = false;
            auto rudder = NAN;
            auto now = clock::now();

            auto slices = can::split(buffer, buffer + size);
            auto p = slices.begin();
            auto ptr = *p;
            for (++p; p != slices.end(); ptr = *p++) {
                using namespace can::pm1;
                constexpr static auto STATE = any_node::state::rx.data.msg_type;
                constexpr static can::header_t MASK{.data{
                    .head = 0xff,
                    .node_type_h = 0b11,
                    .payload = true,
                    .node_type_l = 0b1111,
                    .msg_type = 0xff,
                }};

                auto header = reinterpret_cast<const can::header_t *>(ptr);
                auto node = controller_t{.data{.type = NODE_TYPE(header), .index = header->data.node_index}};
                auto data = ptr + 5;

                if (header->data.msg_type == STATE) {

                    _states[node.key] = *data;
                    release = release || *data != 1;

                } else if (!((header->key ^ any_tcu::current_position::rx.key) & MASK.key)) {

                    if (node.data.index == 0) {
                        int16_t pulse;
                        std::reverse_copy(data, data + sizeof pulse, reinterpret_cast<uint8_t *>(&pulse));
                        rudder = RAD_OF(pulse > 4095 ? 4095 : pulse < -4095 ? -4095
                                                                            : pulse,
                                        default_rudder_k);
                    }

                } else if (!((header->key ^ any_ecu::current_position::rx.key) & MASK.key)) {

                    int32_t pulse;
                    std::reverse_copy(data, data + sizeof pulse, reinterpret_cast<uint8_t *>(&pulse));

                } else if (!((header->key ^ any_vcu::battery_percent::rx.key) & MASK.key))

                    _battery_percent = *data;
            }
            std::memcpy(buffer, ptr, size -= ptr - buffer);
            auto end = buffer += size;
            if (release) {
                *reinterpret_cast<can::header_t *>(end) = can::pm1::every_node::unlock;
                end = to_stream<uint8_t>(end, 0xff);
            }
            if (!std::isnan(rudder)) {
                using namespace std::chrono_literals;

                _rudder_received = now;
                auto should_stop = _rudder_received > _target_set + 200ms;

                if (std::isnan(_target.rudder) || should_stop)
                    _target = {0, rudder};

                if (!should_stop || _internal_speed != 0) {
                    auto optimized = model.optimize(_target, {_internal_speed, rudder});
                    auto wheels = model.to_wheels(optimized);
                    _internal_speed = optimized.speed;

                    *reinterpret_cast<can::header_t *>(end) = can::pm1::ecu<0>::target_speed;
                    end = to_stream<int32_t>(end, PULSES_OF(wheels.left, default_wheel_k));
                    *reinterpret_cast<can::header_t *>(end) = can::pm1::ecu<1>::target_speed;
                    end = to_stream<int32_t>(end, PULSES_OF(wheels.right, default_wheel_k));
                    *reinterpret_cast<can::header_t *>(end) = can::pm1::tcu<0>::target_position;
                    end = to_stream<int16_t>(end, PULSES_OF(_target.rudder, default_rudder_k));
                }
            }
            size = static_cast<uint8_t>(end - buffer);
        }
    };

    chassis_t::chassis_t() : _implement(new implement_t) {}

    chassis_t::~chassis_t() { delete _implement; }

    void chassis_t::communicate(uint8_t *&buffer, uint8_t &size) {
        _implement->communicate(buffer, size);
    }

    bool chassis_t::alive() const {
        return _implement->alive();
    }

    uint8_t chassis_t::battery_percent() const {
        return _implement->battery_percent();
    }

    void chassis_t::set_velocity(float v, float w) {
        _implement->set_target(_implement->model.from_velocity({v, w}));
    }

    void chassis_t::set_physical(float speed, float rudder) {
        _implement->set_target({speed, rudder});
    }

    void chassis_t::close() {
        _implement->close();
    }
}// namespace autolabor::pm1
