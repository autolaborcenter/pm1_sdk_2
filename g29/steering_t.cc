#include "steering_t.hh"

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/epoll.h>
#include <unistd.h>

#include <cstring>
#include <filesystem>
#include <fstream>

class state_t {
    int16_t
        _direction = 0,
        _power = 32767;
    uint8_t
        _level = 1,
        _max_level = 5;

public:
    std::pair<float, float> to_float() const {
        return std::make_pair(_direction / 32768.0f, (32767 - _power) * (_level ? _level : -1) / _max_level / 65536.0f);
    }

    void set_direction(uint16_t value) {
        _direction = value;
    }

    void set_power(uint16_t value) {
        _power = value;
    }

    void level_up() {
        if (_level < _max_level)
            ++_level;
    }

    void level_down() {
        if (_level > 0)
            --_level;
    }
};

class steering_t::context_t {
    std::string _name_event, _name_js;

    int _event, _js, _epoll;

    state_t _state;

    void update_autocenter(uint16_t value) const {
        input_event msg{.type = EV_FF, .code = FF_AUTOCENTER, .value = value};
        auto _ = write(_event, &msg, sizeof(input_event));
    }

    void value_updated(float &speed, float &rudder) const {
        auto [rudder_, speed_] = _state.to_float();
        update_autocenter(0x2000 + 0x4000 * std::abs(speed_));
        speed = speed_;
        rudder = rudder_;
    }

public:
    context_t(const char *event, const char *js)
        : _name_event(event),
          _name_js(js),
          _event(0),
          _js(0),
          _epoll(epoll_create1(0)),
          _state{} {}

    context_t(context_t const &) = delete;
    context_t(context_t &&) noexcept = delete;
    context_t &operator=(context_t const &) = delete;
    context_t &operator=(context_t &&) = delete;

    ~context_t() {
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
        _state = {};
        epoll_ctl(_epoll, EPOLL_CTL_DEL, _js, nullptr);
        ::close(std::exchange(_event, 0));
        ::close(std::exchange(_js, 0));
    }

    bool wait_event(float &speed, float &rudder) {
        while (true) {
            js_event event{};
            epoll_event epoll{};
            auto n = epoll_wait(_epoll, &epoll, 1, 100);
            if (!n)
                return true;
            if (read(epoll.data.u32, &event, sizeof(js_event)) < 0) {
                close();
                return false;
            }

            switch (event.type) {
                case 1:
                    if (event.value == 0)
                        switch (event.number) {
                            case 4:
                            case 19:
                                _state.level_up();
                                value_updated(speed, rudder);
                                return true;
                            case 5:
                            case 20:
                                _state.level_down();
                                value_updated(speed, rudder);
                                return true;
                        }
                    break;
                case 2:
                    switch (event.number) {
                        case 0:
                            _state.set_direction(event.value);
                            value_updated(speed, rudder);
                            return true;
                        case 2:
                            _state.set_power(event.value);
                            value_updated(speed, rudder);
                            return true;
                    }
                    break;
            }
        }
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

bool steering_t::wait_event(float &speed, float &rudder) {
    return _context ? _context->wait_event(speed, rudder) : false;
}

bool wait_event(float &speed, float &rudder) {
    const char *name;
    auto &steering = steering_t::global(name);
    return name ? steering.wait_event(speed, rudder) : false;
}

steering_t &steering_t::global(const char *&name_) {
    constexpr static auto
        N_PREFIX = "N: Name=",
        H_PREFIX = "H: Handlers=",
        FF_PREFIX = "B: FF=";
    constexpr static auto
        N_PREFIX_LEN = std::strlen(N_PREFIX),
        H_PREFIX_LEN = std::strlen(H_PREFIX),
        FF_PREFIX_LEN = std::strlen(FF_PREFIX);

    static std::string name;
    static steering_t steering;

    if (steering.open()) {
        name_ = name.c_str();
        return steering;
    }

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
    char line[128];

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
            if (temp.open()) {
                steering = std::move(temp);
                break;
            } else
                name.clear();
        }

    name_ = steering.open() ? name.c_str() : nullptr;
    return steering;
}

/// constant force

// /* Initialize constant force effect */
// ff_effect effect{
//     .type = FF_CONSTANT,
//     .id = -1,
//     .direction = 0xc000,
//     .trigger{},
//     .replay{.length = 0xffff, .delay = 0},
//     .u{.constant{
//         .level = 10000,
//         .envelope{
//             .attack_length = 0,
//             .attack_level = 0,
//             .fade_length = 0,
//             .fade_level = 0,
//         }}},
// };

// /* Upload effect */
// if (ioctl(event, EVIOCSFF, &effect) < 0)
// {
//     fprintf(stderr, "ERROR: uploading effect failed (%s) [%s:%d]\n", strerror(errno), __FILE__, __LINE__);
//     return 1;
// }

// /* Start effect */
// input_event msg{
//     .time{},
//     .type = EV_FF,
//     .code = static_cast<uint16_t>(effect.id),
//     .value = 1,
// };
// if (write(event, &msg, sizeof(msg)) != sizeof(msg))
// {
//     std::cerr << "failed to start effect: " << strerror(errno) << std::endl;
//     return 1;
// }
