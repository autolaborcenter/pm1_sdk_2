#ifdef WIN32

#include "serial_port_t.hh"

#include <windows.h>

serial_port_t::
serial_port_t(std::string name)
    : _name(std::move(name)),
      _fd(nullptr) {}

serial_port_t::
serial_port_t(serial_port_t&& others) noexcept
    : _name(std::move(others._name)),
      _fd(std::exchange(others._fd, nullptr)) {}

serial_port_t&
serial_port_t::
operator=(serial_port_t&& others) noexcept {
    if (this != &others)
    {
        CloseHandle(_fd);
        _name = std::move(others._name);
        _fd = std::exchange(others._fd, nullptr);
    }
    return *this;
}

serial_port_t::
~serial_port_t() {
    if (_fd)
        CloseHandle(_fd);
}

const char *
serial_port_t::
name() const {
    return _name.c_str();
}

int 
serial_port_t::
open() {
    if (_fd)
        return 0;

    auto path = new char[_name.size() + 5]{};
    std::strcpy(path, R"(\\.\)");
    std::strcpy(path + 4, _name.c_str());
    auto fd = CreateFile(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    delete[] path;
    if (fd == INVALID_HANDLE_VALUE)
        return GetLastError();

    DCB dcb{ .DCBlength = sizeof(DCB) };

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;

    if (!SetCommState(fd, &dcb))
    {
        auto e = GetLastError();
        CloseHandle(fd);
        return e;
    }

    COMMTIMEOUTS timeouts{};

    _fd = fd;
    return 0;
}

void
serial_port_t::
close() {
    if (!_fd)
        return;
    CloseHandle(_fd);
    _fd = 0;
}

size_t
serial_port_t::
read(void* data, size_t size) const {
    DWORD result;
    return !_fd || ReadFile(_fd, data, size, &result, NULL) ? result : 0;
}

size_t
serial_port_t::
write(void* data, size_t size) const {
    DWORD result;
    return !_fd || WriteFile(_fd, data, size, &result, NULL) ? result : 0;
}

#endif // WIN32
