#include "steering_t.hh"

extern "C" {
#include "control_model/model.h"
}

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/epoll.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>

#include <memory>
#include <shared_mutex>

/** int16_t to float */
class g29_value_t {
    int16_t
        _direction = 0,
        _power = 32767;
    uint8_t
        _level = 1,
        _max_level = 5;

public:
    void to_float(float &speed, float &rudder) const {
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
        if (_level < _max_level) {
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

class steering_t::context_t {
    std::string _name_event, _name_js;

    int _event, _js, _epoll;

    using clock = std::chrono::steady_clock;
    clock::time_point _state_updated;

    g29_value_t _value;
    chassis_config_t _chassis = default_config;

    void update_autocenter(uint16_t value) const {
        input_event msg{.type = EV_FF, .code = FF_AUTOCENTER, .value = value};
        auto _ = write(_event, &msg, sizeof(input_event));
    }

    void update_ff(physical p, bool actual) {
        if (actual)
            _state_updated = clock::now();
        else if (_state_updated != clock::time_point::min() &&
                 clock::now() > _state_updated + std::chrono::seconds(1))
            p = physical_zero;
        update_autocenter(0x2800 + 0x6000 * std::abs(physical_to_velocity(p, &_chassis).v));
    }

public:
    context_t(const char *event, const char *js)
        : _name_event(event),
          _name_js(js),
          _event(0),
          _js(0),
          _epoll(epoll_create1(0)),
          _value{},
          _state_updated(decltype(_state_updated)::min()) {}

    context_t(context_t const &) = delete;
    context_t(context_t &&) noexcept = delete;
    context_t &operator=(context_t const &) = delete;
    context_t &operator=(context_t &&) = delete;

    ~context_t() {
        _value = {};
        ::close(_epoll);
        ::close(_event);
        ::close(_js);
    }

    bool open() {
        if (_event)
            return true;

        std::filesystem::path js_path("/dev/input/");
        auto js = ::open(js_path.append(_name_js).c_str(), O_RDONLY);
        if (js <= 0)
            return false;

        std::filesystem::path event_path("/dev/input/");
        auto event = ::open(event_path.append(_name_event).c_str(), O_RDWR);
        if (event <= 0)
            return false;

        uint8_t ff_bits[FF_MAX / 8 + 1]{};
        if (ioctl(event, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0)
            return false;

#define testBit(bit, array) ((array[bit / 8] >> bit % 8) & 1)
        if (!testBit(FF_CONSTANT, ff_bits) || !testBit(FF_AUTOCENTER, ff_bits))
            return false;
#undef testBit

        _event = event;
        _js = js;
        epoll_event epoll{.events = EPOLLIN, .data{.u32 = static_cast<uint32_t>(js)}};
        epoll_ctl(_epoll, EPOLL_CTL_ADD, js, &epoll);
        update_autocenter(0x8000);

        return true;
    }

    void close() {
        if (!_event)
            return;
        _value = {};
        epoll_ctl(_epoll, EPOLL_CTL_DEL, _js, nullptr);
        ::close(std::exchange(_event, 0));
        ::close(std::exchange(_js, 0));
    }

    bool wait_event(float &speed, float &rudder, int timeout) {
        while (true) {
            js_event event{};
            epoll_event epoll{};
            auto n = epoll_wait(_epoll, &epoll, 1, timeout);
            if (!n) {
                _value.to_float(speed, rudder);
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
                                    _value.to_float(speed, rudder);
                                    update_ff({speed, rudder}, false);
                                }
                                return true;
                            case 5:
                            case 20:
                                if (_value.level_down()) {
                                    _value.to_float(speed, rudder);
                                    update_ff({speed, rudder}, false);
                                }
                                return true;
                        }
                    break;
                case 2:
                    switch (event.number) {
                        case 0:
                            _value.set_direction(event.value);
                            _value.to_float(speed, rudder);
                            update_ff({speed, rudder}, false);
                            return true;
                        case 1:
                            if (_value.sternway(event.value)) {
                                _value.to_float(speed, rudder);
                                update_ff({speed, rudder}, false);
                                return true;
                            }
                            break;
                        case 2:
                            _value.set_power(event.value);
                            _value.to_float(speed, rudder);
                            update_ff({speed, rudder}, false);
                            return true;
                    }
                    break;
            }
        }
    }

    void set_state(float speed, float rudder) {
        update_ff({speed, rudder}, true);
    }
};

steering_t::steering_t() : _context(nullptr) {}
steering_t::steering_t(const char *event, const char *js) : _context(new context_t(event, js)) {}
steering_t::steering_t(steering_t &&others) noexcept : _context(std::exchange(others._context, nullptr)) {}

steering_t &steering_t::operator=(steering_t &&others) noexcept {
    delete std::exchange(_context, std::exchange(others._context, nullptr));
    return *this;
}

steering_t::~steering_t() {
    delete _context;
}

bool steering_t::open() {
    return _context ? _context->open() : false;
}

void steering_t::close() {
    if (_context)
        _context->close();
}

bool steering_t::wait_event(float &speed, float &rudder, int timeout) {
    return _context ? _context->wait_event(speed, rudder, timeout) : false;
}

void steering_t::set_state(float speed, float rudder) {
    if (_context)
        _context;
}

std::shared_ptr<steering_t> steering();
std::atomic<physical> _target;

bool wait_event(float &speed, float &rudder, int timeout) {
    auto _steering = steering();
    if (_steering && _steering->wait_event(speed, rudder, timeout)) {
        _target = {speed, rudder};
        return true;
    } else {
        _target = physical_zero;
        return false;
    }
}

void set_state(float &speed, float &rudder) {
    auto _steering = steering();
    if (_steering) {
        _steering->set_state(speed, rudder);
        auto target = _target.load();
        speed = target.speed;
        rudder = target.rudder;
    } else {
        _target = physical_zero;
        speed = 0;
        rudder = NAN;
    }
}

void next_pose(float &x, float &y, float &theta) {
}

std::shared_ptr<steering_t> steering() {
    constexpr static auto
        N_PREFIX = "N: Name=",
        H_PREFIX = "H: Handlers=",
        FF_PREFIX = "B: FF=";
    constexpr static auto
        N_PREFIX_LEN = std::strlen(N_PREFIX),
        H_PREFIX_LEN = std::strlen(H_PREFIX),
        FF_PREFIX_LEN = std::strlen(FF_PREFIX);

    static std::shared_ptr<steering_t> _steering;
    static std::string name;
    static char line[512];
    static std::shared_mutex looking_for;

    std::shared_lock<decltype(looking_for)> lock(looking_for);
    if (_steering && _steering->open())
        return _steering;

    std::unique_lock<decltype(looking_for)> retry(looking_for);
    if (_steering && _steering->open())
        return _steering;

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
    std::string event, js;

    while (file.getline(line, sizeof line))
        if (std::strlen(line) < 3) {
            name.clear();
            event.clear();
            js.clear();
        } else if (std::strncmp(line, N_PREFIX, N_PREFIX_LEN) == 0)
            name = line + N_PREFIX_LEN;
        else if (std::strncmp(line, H_PREFIX, H_PREFIX_LEN) == 0) {
            std::string temp;
            std::stringstream stream;
            stream << line + H_PREFIX_LEN;
            while (stream >> temp)
                if (temp.starts_with("event"))
                    event = std::move(temp);
                else if (temp.starts_with("js"))
                    js = std::move(temp);
        } else if (std::strncmp(line, FF_PREFIX, FF_PREFIX_LEN) == 0 &&
                   !name.empty() &&
                   !event.empty() &&
                   !js.empty()) {
            steering_t temp(event.c_str(), js.c_str());
            if (temp.open())
                return _steering = std::make_shared<steering_t>(std::move(temp));
            name.clear();
        }

    return _steering = nullptr;
}
