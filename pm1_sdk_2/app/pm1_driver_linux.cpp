#include "../src/serial_port/serial_port_t.hh"

#include <filesystem>
#include <iostream>

int main()
{
    std::vector<serial_port_t> result;
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/tty"))
        if (entry.is_directory())
        {
            auto name = entry.path().filename().string();
            if (name.starts_with("ttyS") || name.starts_with("ttyUSB") || name.starts_with("ttyACM"))
                result.emplace_back(std::move(name));
        }

    for (auto &port : result)
        std::cout << port.name() << ": " << std::boolalpha << (port.open() == 0) << std::endl;
    return 0;
}
