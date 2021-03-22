#include "pm1_driver_common.h"

#include <fcntl.h>   // open
#include <unistd.h>  // close
#include <termios.h> // config

#include <vector>
#include <filesystem>

using namespace autolabor::pm1;

int main()
{
    std::vector<std::string> ports;
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/tty"))
        if (entry.is_directory())
        {
            auto name = entry.path().filename().string();
            if (name.starts_with("ttyUSB") || name.starts_with("ttyACM"))
                if (name != "ttyACM0")
                    ports.emplace_back(std::move(name));
        }

    std::unordered_map<std::string, chassis_t> chassis;
    std::mutex mutex;
    std::condition_variable signal;
    for (const auto &name : ports)
    {
        std::filesystem::path path("/dev");
        path.append(name);
        auto fd = open(path.c_str(), O_RDWR);
        if (fd <= 0)
            continue;

        // @see https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
        termios tty{};
        cfsetspeed(&tty, B115200);
        tty.c_cflag |= CS8;            // 8 bits per byte
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_cc[VTIME] = 5; // Wait for up to 500ms, returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(fd, TCSAFLUSH, &tty))
        {
            close(fd);
            continue;
        }

        using HANDLE = decltype(fd);
        auto map_iterator = chassis.try_emplace(name).first;
        std::shared_ptr<HANDLE> fd_ptr(
            new HANDLE(fd),
            [map_iterator, &chassis, &mutex, &signal](auto p) {
                close(*p);
                delete p;
                std::unique_lock<std::mutex> lock(mutex);
                chassis.erase(map_iterator);
                if (chassis.empty())
                {
                    lock.unlock();
                    signal.notify_all();
                }
            });

        std::thread([ptr = &map_iterator->second, fd_ptr] {
            uint8_t buffer[64];
            uint8_t size;
            do
            {
                auto n = read(*fd_ptr, buffer + size, sizeof buffer - size);
                if (n <= 0)
                    break;
                auto [i, j] = ptr->communicate(buffer, size + n);
                size = i;
                if (j > i && write(*fd_ptr, buffer + i, j - i) <= 0)
                    break;
            } while (ptr->alive());
            ptr->close();
        }).detach();

        std::thread([ptr = &map_iterator->second, fd_ptr] {
            auto msg = loop_msg_t();
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0; ptr->alive(); ++i)
            {
                auto [buffer, size] = msg[i];
                if (write(*fd_ptr, buffer, size) <= 0)
                    break;
                delete[] buffer;
                std::this_thread::sleep_until(t0 += loop_msg_t::PERIOD);
            }
            ptr->close();
        }).detach();
    }

    launch_parser(mutex, signal, chassis).detach();

    std::unique_lock<std::mutex> lock(mutex);
    while (!chassis.empty())
        signal.wait(lock);

    return 0;
}
