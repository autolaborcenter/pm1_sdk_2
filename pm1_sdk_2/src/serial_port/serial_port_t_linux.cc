#ifdef __linux
#include "serial_port_t.hh"

#include <fcntl.h>   // open
#include <unistd.h>  // close
#include <termios.h> // config

#include <sstream>
#include <filesystem>

static int baud_rate(uint32_t);
static int ms_to_ds(uint32_t);

serial_port_t::serial_port_t(std::string name)
    : _name(std::move(name)),
      _fd(0),
      _timeout(1) {}

serial_port_t::serial_port_t(serial_port_t &&others) noexcept
    : _name(std::move(others._name)),
      _timeout(std::exchange(others._timeout, 0))
{
    ::close(std::exchange(_fd, std::exchange(others._fd, 0)));
}

serial_port_t &serial_port_t::operator=(serial_port_t &&others) noexcept
{
    if (this != &others)
    {
        ::close(_fd);
        _name = std::move(others._name);
        _fd = std::exchange(others._fd, 0);
        _timeout = std::exchange(others._timeout, 0);
    }
    return *this;
}

serial_port_t::
    ~serial_port_t()
{
    ::close(_fd);
}

const char *serial_port_t::name() const
{
    return _name.c_str();
}

int serial_port_t::open()
{
    if (_fd)
        return 0;

    std::filesystem::path path("/dev");
    path.append(_name);
    auto fd = ::open(path.c_str(), O_RDWR);
    if (fd <= 0)
        return errno;

    /*
     * SEE https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
     */

    // 千言万语汇成一句话：全清零
    termios tty{};

    // if (tcgetattr(fd, &tty))
    //     return errno;

    tty.c_cc[VTIME] = ms_to_ds(_timeout);
    tty.c_cc[VMIN] = 0;

    cfsetspeed(&tty, B115200);

    // tty.c_cflag &= ~PARENB;        // Clear parity bit
    // tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
    // tty.c_cflag &= ~CSIZE;         // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8; // 8 bits per byte
    // tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // tty.c_lflag &= ~ICANON; // nothing special with new-line character
    // tty.c_lflag &= ~ECHO;   // Disable echo
    // tty.c_lflag &= ~ECHOE;  // Disable erasure
    // tty.c_lflag &= ~ECHONL; // Disable new-line echo
    // tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    // tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    // tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    // tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    // tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSAFLUSH, &tty))
    {
        auto e = errno;
        ::close(fd);
        return e;
    }

    _fd = fd;
    return 0;
}

void serial_port_t::close()
{
    if (!_fd)
        return;
    ::close(_fd);
    _fd = 0;
}

size_t serial_port_t::read(void *data, size_t size) const
{
    return _fd ? ::read(_fd, data, size) : 0;
}

size_t serial_port_t::write(void *data, size_t size) const
{
    return _fd ? ::write(_fd, data, size) : 0;
}

#define SET_ONLINE(STRUCT, WHAT)                \
    if (_fd)                                    \
    {                                           \
        termios STRUCT;                         \
                                                \
        if (tcgetattr(_fd, &STRUCT))            \
            return errno;                       \
                                                \
        WHAT;                                   \
                                                \
        if (tcsetattr(_fd, TCSAFLUSH, &STRUCT)) \
            return errno;                       \
    }

int serial_port_t::set_timeout(uint32_t ms)
{
    SET_ONLINE(tty, tty.c_cc[VTIME] = ms_to_ds(ms))
    _timeout = ms;
    return 0;
}

#undef SET_ONLINE

int ms_to_ds(uint32_t ms)
{
    return ms ? (ms + 50) / 100 : 0;
}

#endif // __linux
