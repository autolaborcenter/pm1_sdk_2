#include "pm1_driver_common.h"

#include <filesystem>
#include <iostream>

int main()
{
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/tty"))
        if (entry.is_directory())
        {
            auto name = entry.path().filename().string();
            if (name.starts_with("ttyUSB") || name.starts_with("ttyACM"))
                std::cout << name << std::endl;
        }
    return 0;
}
