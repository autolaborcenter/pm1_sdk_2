#include <unistd.h>

#include <cstring>
#include <string>
#include <utility>

class servo_t {
    constexpr static auto CMD = "#000P0000T0000!";

    int _fd;
    char buffer[std::strlen(CMD) + 1]{};

public:
    servo_t(int fd = -1) : _fd(-1) {
        constexpr static auto VERSION_ASK = "#000PVER!";
        constexpr static auto VERSION_ASK_LEN = std::strlen(VERSION_ASK);

        constexpr static auto VERSION_ACK = "#000P ZServo V3.45H!";
        constexpr static auto VERSION_ACK_LEN = std::strlen(VERSION_ACK);

        if (fd >= 0 && VERSION_ASK_LEN == write(fd, VERSION_ASK, VERSION_ASK_LEN)) {
            char buffer[VERSION_ACK_LEN + 1]{};
            auto end = 0;
            while (end < sizeof(buffer)) {
                auto n = read(fd, buffer + end, sizeof(buffer) - end);
                if (n <= 0) break;
                if ((end += n) == VERSION_ACK_LEN && std::strcmp(buffer, VERSION_ACK) == 0) {
                    _fd = fd;
                    strcpy(buffer, CMD);
                    return;
                }
            }
        }
        close(fd);
    }
    servo_t(servo_t const &) = delete;
    servo_t(servo_t &&others) noexcept : _fd(std::exchange(others._fd, -1)) {
        strcpy(buffer, CMD);
    }

    servo_t &operator=(servo_t const &) = delete;
    servo_t &operator=(servo_t &&others) noexcept {
        if (this != &others) {
            close(std::exchange(_fd, std::exchange(others._fd, -1)));
            strcpy(buffer, CMD);
        }
        return *this;
    }

    ~servo_t() { close(_fd); }

    operator bool() const { return _fd >= 0; }

    void operator()(int value) {
        if (_fd >= 0 && 500 <= value && value <= 2500) {
            buffer[5] = '0';
            auto text = std::to_string(value);
            text.copy(buffer + 9 - text.size(), text.size());
            if (write(_fd, buffer, sizeof(buffer) - 1) != sizeof(buffer) - 1)
                close(std::exchange(_fd, -1));
        }
    }
};