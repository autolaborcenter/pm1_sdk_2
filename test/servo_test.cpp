#include "servo.hpp"

#include <fcntl.h>  // open
#include <termios.h>// config
#include <unistd.h> // close

#include <filesystem>
#include <iostream>
#include <vector>

int main() {
    std::vector<std::string> ports;
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/tty"))
        if (entry.is_directory()) {
            auto name = entry.path().filename().string();
            if (name.starts_with("ttyUSB") || name.starts_with("ttyACM"))
                if (name != "ttyACM0")
                    ports.emplace_back(std::move(name));
        }

    servo_t servo(-1);
    for (const auto &name : ports) {
        std::filesystem::path path("/dev");
        auto fd = open(path.append(name).c_str(), O_RDWR);
        if (fd < 0) continue;

        // @see https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
        termios tty{};
        cfsetspeed(&tty, B115200);
        tty.c_cflag |= CS8;           // 8 bits per byte
        tty.c_cflag |= CREAD | CLOCAL;// Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_cc[VTIME] = 5;// Wait for up to 500ms, returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(fd, TCSAFLUSH, &tty)) {
            close(fd);
            continue;
        }
        servo_t temp(fd);
        if (temp) {
            servo = std::move(temp);
            std::cout << "servo name = " << name << std::endl;
            break;
        }
    }
    std::string line;
    if (servo)
        while (std::getline(std::cin, line))
            try {
                servo(std::stoi(line));
            } catch (...) {
            }

    return 0;
}
