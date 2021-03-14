#ifndef SERIAL_PORT_T_H
#define SERIAL_PORT_T_H

#include <vector>
#include <string>

class serial_port_t
{
    using handle_t =
#if defined __linux
int;
#elif defined WIN32
void *;
    #endif // handle_t
    
    std::string _name;
    handle_t _fd;

public:
    explicit serial_port_t(std::string);
    
    serial_port_t(const serial_port_t &) = delete;
    
    serial_port_t(serial_port_t &&) noexcept;
    
    serial_port_t &operator=(const serial_port_t &) = delete;
    
    serial_port_t &operator=(serial_port_t &&) noexcept;
    
    ~serial_port_t();
    
    [[nodiscard]] const char *name() const;
    
    [[nodiscard]] bool is_open() const;
    
    auto open();
    
    void close();
    
    size_t read(void *, size_t) const;
    
    size_t write(void *, size_t) const;
};

#endif // !SERIAL_PORT_T_H
