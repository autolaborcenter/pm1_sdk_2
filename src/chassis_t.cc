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
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace autolabor::pm1 {
    class differential_t {
        int32_t _cache[2][2]{};

        bool _initialized = false, _ready[2]{};

    public:
        constexpr static auto L = 0, R = 1;

        void reset() {
            _initialized = _ready[L] = _ready[R] = false;
        }

        void update() {
            _ready[L] = _ready[R] = false;
        }

        std::optional<std::pair<int32_t, int32_t>> update(int which, int32_t value) {
            if (which > 1) return std::nullopt;
            const auto other = 1 - which;

            _cache[which][1] = value;

            if (_ready[other]) {
                // 收齐
                _ready[other] = false;
                // 差分
                std::pair<int32_t, int32_t> result{_cache[0][1] - _cache[0][0], _cache[1][1] - _cache[1][0]};
                _cache[0][0] = _cache[0][1];
                _cache[1][0] = _cache[1][1];

                if (!_initialized)
                    _initialized = true;
                else
                    return result;
            } else
                // 未收齐
                _ready[which] = true;

            return std::nullopt;
        }
    };

    class chassis_t::implement_t {
        using clock = std::chrono::steady_clock;

        union controller_t {
            uint16_t key;
            struct {
                uint8_t type, index;
            } data;
        };

        uint8_t _bytes_to_send[24], _times_to_send;

        std::unordered_map<decltype(controller_t::key), uint8_t> _states;
        uint8_t _battery_percent;

        clock::time_point _target_set, _rudder_received, _active_send;
        physical _current, _target;
        bool _alive;

        std::mutex _mutex;
        differential_t _differential;
        odometry_t<> _odometry;

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
              _active_send(_rudder_received - model.period()),
              _times_to_send(0),
              _current(physical_zero),
              _target(physical_zero),
              _odometry{},
              _alive(true) {
#define SET_HEADER(N) *reinterpret_cast<can::header_t *>(_bytes_to_send + (N)) = can::pm1
            SET_HEADER(0)::every_tcu::current_position::tx;
            SET_HEADER(6)::every_ecu::current_position::tx;
            SET_HEADER(12)::every_node::state::tx;
            SET_HEADER(18)::every_vcu::battery_percent::tx;
#undef SET_HEADER
            for (auto i = 0; i < sizeof(_bytes_to_send); i += 6)
                _bytes_to_send[i + 5] = can::crc_calculate(_bytes_to_send + i + 1, _bytes_to_send + i + 5);
        }

        bool alive() const { return _alive && clock::now() < _rudder_received + std::chrono::milliseconds(200); }
        uint8_t battery_percent() const { return _battery_percent; }
        physical current() const { return _current; }
        physical target() const { return _target; }
        odometry_t<> odometry() const { return _odometry; }

        active_sending_t next_to_send() {
            auto now = clock::now();
            while (_active_send < now) {
                _active_send += model.period();
                ++_times_to_send;
            }
            using namespace std::chrono_literals;
            uint8_t size = _times_to_send % (10s / model.period()) == 0
                               ? 24
                           : _times_to_send % (400ms / model.period()) == 0
                               ? 18
                           : _times_to_send % 2 == 0
                               ? 12
                               : 6;
            if (size > 6) {
                std::lock_guard<decltype(_mutex)> lock(_mutex);
                _differential.update();
            }
            return {_active_send, _bytes_to_send, size};
        }

        void set_target(physical target) {
            _target_set = clock::now();
            _target = target;
        }

        void close() { _alive = false; }

        void communicate(uint8_t *&buffer, uint8_t &size) {
            auto locked = false;
            auto rudder = NAN;
            auto now = clock::now();
            // 解析
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
                    locked = locked || *data != 1;

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
                    std::unique_lock<decltype(_mutex)> lock(_mutex);
                    auto d = _differential.update(node.data.index, pulse);
                    lock.unlock();
                    if (d) {
                        auto [l, r] = *d;
                        _odometry += model.to_delta(l, r);
                    }

                } else if (!((header->key ^ any_vcu::battery_percent::rx.key) & MASK.key))

                    _battery_percent = *data;
            }
            // 拷贝未解析的字节
            std::memcpy(buffer, ptr, size -= ptr - buffer);
            auto end = buffer += size;
            // 解锁
            if (locked) {
                *reinterpret_cast<can::header_t *>(end) = can::pm1::every_node::unlock;
                end = to_stream<uint8_t>(end, 0xff);
            }
            // 控制
            if (!std::isnan(rudder)) {
                using namespace std::chrono_literals;
                _current.rudder = rudder;

                _rudder_received = now;
                auto should_stop = _rudder_received > _target_set + 200ms;

                if (std::isnan(_target.rudder) || should_stop)
                    _target = {0, rudder};

                if (!should_stop || _current.speed != 0) {
                    _current.speed = model.optimize(_target, _current).speed;
                    auto wheels = model.to_wheels(_current);

                    *reinterpret_cast<can::header_t *>(end) = can::pm1::ecu<0>::target_speed;
                    end = to_stream<int32_t>(end, PULSES_OF(wheels.left, default_wheel_k));
                    *reinterpret_cast<can::header_t *>(end) = can::pm1::ecu<1>::target_speed;
                    end = to_stream<int32_t>(end, PULSES_OF(wheels.right, default_wheel_k));
                    *reinterpret_cast<can::header_t *>(end) = can::pm1::tcu<0>::target_position;
                    end = to_stream<int16_t>(end, PULSES_OF(_target.rudder, default_rudder_k));
                }
            }
            // 传出写字节长度
            size = static_cast<uint8_t>(end - buffer);
        }
    };

    chassis_t::chassis_t() : _implement(new implement_t) {}
    chassis_t::chassis_t(chassis_t &&others) noexcept : _implement(std::exchange(others._implement, nullptr)) {}
    chassis_t::~chassis_t() { delete _implement; }

    void chassis_t::communicate(uint8_t *&buffer, uint8_t &size) { _implement->communicate(buffer, size); }

    bool chassis_t::alive() const { return _implement->alive(); }
    uint8_t chassis_t::battery_percent() const { return _implement->battery_percent(); }
    physical chassis_t::current() const { return _implement->current(); }
    physical chassis_t::target() const { return _implement->target(); }
    odometry_t<> chassis_t::odometry() const { return _implement->odometry(); }

    chassis_t::active_sending_t chassis_t::next_to_send() { return _implement->next_to_send(); }
    void chassis_t::set_target(physical state) { _implement->set_target(state); }
    void chassis_t::close() { _implement->close(); }
}// namespace autolabor::pm1
