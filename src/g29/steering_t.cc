#include "steering_t.hh"

extern "C" {
#include "src/control_model/model.h"
#include "src/control_model/optimization.h"
}

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/epoll.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>

#include <algorithm>
#include <mutex>
#include <optional>

namespace logitech {
    /** int16_t to float */
    class g29_value_t {
        int16_t
            _direction = 0,
            _power = 32767;
        uint8_t
            _level = 1,
            _max_level = 5;

    public:
        void get(uint8_t &level, float &speed, float &rudder) const {
            level = _level;
            speed = (32767 - _power) * (_level ? _level : -1) / _max_level / 65536.0f;
            rudder = std::copysignf(std::pow(std::abs(_direction) / 32768.0f, 2.0f), _direction) * pi_f / 2;
        }

        void set_direction(uint16_t value) {
            _direction = value;
        }

        void set_power(uint16_t value) {
            _power = value;
        }

        bool level_up() {
            if (0 < _level && _level < _max_level) {
                ++_level;
                return true;
            }
            return false;
        }

        bool level_down() {
            if (_level > 1) {
                --_level;
                return true;
            }
            return false;
        }

        bool sternway(int16_t value) {
            if (!_level && value > 16384) {
                _level = 1;
                return true;
            }
            if (_level && value < -16384) {
                _level = 0;
                return true;
            }
            return false;
        }
    };

    class steering_t::implement_t {
        int _event, _js, _epoll;

        g29_value_t _value;
        chassis_config_t _chassis = default_config;

        void update_autocenter(uint16_t value) const {
            input_event msg{.type = EV_FF, .code = FF_AUTOCENTER, .value = value};
            std::ignore = write(_event, &msg, sizeof(input_event));
        }

        void update_ff(physical p) {
            update_autocenter(0x2800 + 0x4000 * std::abs(physical_to_velocity(p, &_chassis).v));
        }

    public:
        implement_t(const char *name_event, const char *name_js)
            : _event(-1),
              _js(-1),
              _epoll(epoll_create1(0)),
              _value{} {
            if (!name_event || !name_js) return;

            std::filesystem::path js_path("/dev/input/");
            std::filesystem::path event_path("/dev/input/");
            auto js = open(js_path.append(name_js).c_str(), O_RDONLY);
            auto event = open(event_path.append(name_event).c_str(), O_RDWR);
            if (event < 0) {
                ::close(js);
                ::close(event);
                return;
            }

#define testBit(bit, array) ((array[bit / 8] >> bit % 8) & 1)
            uint8_t ff_bits[FF_MAX / 8 + 1]{};
            if (ioctl(event, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0 ||
                !testBit(FF_CONSTANT, ff_bits) ||
                !testBit(FF_AUTOCENTER, ff_bits)) {
                ::close(js);
                ::close(event);
                return;
            }
#undef testBit

            _event = event;
            _js = js;
            epoll_event epoll{.events = EPOLLIN, .data{.u32 = static_cast<uint32_t>(js)}};
            epoll_ctl(_epoll, EPOLL_CTL_ADD, js, &epoll);
            update_autocenter(0x8000);
        }

        ~implement_t() {
            _value = {};
            ::close(_epoll);
            ::close(_event);
            ::close(_js);
        }

        operator bool() const { return _event >= 0; }

        void close() {
            _value = {};
            epoll_ctl(_epoll, EPOLL_CTL_DEL, _js, nullptr);
            ::close(std::exchange(_event, -1));
            ::close(std::exchange(_js, -1));
            ::close(std::exchange(_epoll, -1));
        }

        bool wait_event(uint8_t &level, float &speed, float &rudder, int timeout) {
            while (true) {
                js_event event{};
                epoll_event epoll{};
                auto n = epoll_wait(_epoll, &epoll, 1, timeout);
                if (!n) {
                    _value.get(level, speed, rudder);
                    return true;
                }
                if (read(epoll.data.u32, &event, sizeof(js_event)) < 0) {
                    close();
                    return false;
                }

                switch (event.type) {
                    case 1:
                        if (!event.value)
                            switch (event.number) {
                                case 4:
                                case 19:
                                    if (_value.level_up()) {
                                        _value.get(level, speed, rudder);
                                        update_ff({speed, rudder});
                                    }
                                    return true;
                                case 5:
                                case 20:
                                    if (_value.level_down()) {
                                        _value.get(level, speed, rudder);
                                        update_ff({speed, rudder});
                                    }
                                    return true;
                            }
                        break;
                    case 2:
                        switch (event.number) {
                            case 0:
                                _value.set_direction(event.value);
                                _value.get(level, speed, rudder);
                                update_ff({speed, rudder});
                                return true;
                            case 1:
                                if (_value.sternway(event.value)) {
                                    _value.get(level, speed, rudder);
                                    update_ff({speed, rudder});
                                    return true;
                                }
                                break;
                            case 2:
                                _value.set_power(event.value);
                                _value.get(level, speed, rudder);
                                update_ff({speed, rudder});
                                return true;
                        }
                        break;
                }
            }
        }
    };

    steering_t::steering_t(const char *event, const char *js) : _implement(new implement_t(event, js)) {}
    steering_t::steering_t(steering_t &&others) noexcept : _implement(std::exchange(others._implement, nullptr)) {}
    steering_t::~steering_t() { delete _implement; }

    steering_t::operator bool() const { return _implement->operator bool(); }
    void steering_t::close() { _implement->close(); }

    bool steering_t::wait_event(uint8_t &level, float &speed, float &rudder, int timeout) {
        return _implement->wait_event(level, speed, rudder, timeout);
    }

    steering_t steering_t::scan() {
        constexpr static auto
            N_PREFIX = "N: Name=",
            H_PREFIX = "H: Handlers=",
            FF_PREFIX = "B: FF=";
        constexpr static auto
            N_PREFIX_LEN = std::strlen(N_PREFIX),
            H_PREFIX_LEN = std::strlen(H_PREFIX),
            FF_PREFIX_LEN = std::strlen(FF_PREFIX);

        // I: Bus=0003 Vendor=046d Product=c24f Version=0111
        // N: Name="Logitech G29 Driving Force Racing Wheel"
        // P: Phys=usb-0000:02:00.0-2.2/input0
        // S: Sysfs=/devices/pci0000:00/0000:00:11.0/0000:02:00.0/usb2/2-2/2-2.2/2-2.2:1.0/0003:046D:C24F.0003/input/input7
        // U: Uniq=
        // H: Handlers=event5 js0
        // B: PROP=0
        // B: EV=20001b
        // B: KEY=1ff 0 0 0 0 0 0 ffff00000000 0 0 0 0
        // B: ABS=30027
        // B: MSC=10
        // B: FF=300040000 0

        std::ifstream file("/proc/bus/input/devices");
        std::string line, name, event, js;

        while (std::getline(file, line))
            if (line.size() < 3) {
                name.clear();
                event.clear();
                js.clear();
            } else if (line.starts_with(N_PREFIX))
                name = line.substr(N_PREFIX_LEN);
            else if (line.starts_with(H_PREFIX)) {
                std::string temp;
                std::stringstream stream(line.substr(H_PREFIX_LEN));
                while (stream >> temp)
                    if (temp.starts_with("event"))
                        event = std::move(temp);
                    else if (temp.starts_with("js"))
                        js = std::move(temp);
            } else if (line.starts_with(FF_PREFIX) && !name.empty() && !event.empty() && !js.empty()) {
                steering_t device(event.c_str(), js.c_str());
                if (device) return device;
                name.clear();
            }

        return steering_t(nullptr, nullptr);
    }
}// namespace logitech
