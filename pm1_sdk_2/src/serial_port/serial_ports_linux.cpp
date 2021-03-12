#ifdef __linux

#include "serial_port_t.hh"

#include <filesystem>

std::vector<serial_port_t>
serial_port_t::
serial_ports()
{
    std::vector<serial_port_t> result;
    for (const auto& entry : std::filesystem::directory_iterator("/sys/class/tty"))
        if (entry.is_directory())
        {
            auto name = entry.path().filename().string();
            if (name.starts_with("ttyS") || name.starts_with("ttyUSB") || name.starts_with("ttyACM"))
                result.push_back(std::move(name));
        }
    return result;
}

#endif