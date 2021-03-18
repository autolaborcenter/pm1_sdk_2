#include "../src/chassis_t.hh"
#include "../src/autocan/pm1.h"

#include <Windows.h>
#include <SetupAPI.h>

#pragma comment (lib, "setupapi.lib")

#include <mutex>
#include <thread>

using namespace autolabor;

static void WINAPI write_callback(DWORD, DWORD, LPOVERLAPPED);

static void WINAPI read_callback(DWORD, DWORD, LPOVERLAPPED);

struct read_context_t {
    pm1::chassis_t *chassis;
    HANDLE handle;
    uint8_t buffer[128];
    uint8_t size;
};

int main() {
    std::vector<std::string> ports;
    {
        auto set = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
        
        if (set == INVALID_HANDLE_VALUE)
            return 1;
    
        SP_DEVINFO_DATA data{.cbSize = sizeof(SP_DEVINFO_DATA)};
        char buffer[64];
        for (auto i = 0; SetupDiEnumDeviceInfo(set, i, &data); ++i) {
            SetupDiGetDeviceRegistryProperty(set, &data, SPDRP_FRIENDLYNAME, NULL, reinterpret_cast<PBYTE>(buffer), sizeof(buffer), NULL);
            ports.emplace_back(buffer);
        }
    
        SetupDiDestroyDeviceInfoList(set);
    }
    
    std::atomic_size_t port_count = 0;
    std::condition_variable signal;
    for (const auto &name : ports) {
        auto p = name.find("COM");
        if (p < 0 || p >= name.size())
            continue;
        auto q = p + 2;
        while (std::isdigit(name[++q]));
        
        auto path = R"(\\.\)" + name.substr(p, q - p);
        
        auto fd = CreateFile(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
        if (fd == INVALID_HANDLE_VALUE)
            continue;
        
        DCB dcb{
            .DCBlength = sizeof(DCB),
            .BaudRate  = CBR_115200,
            .ByteSize  = 8,
        };
        if (!SetCommState(fd, &dcb)) {
            CloseHandle(fd);
            continue;
        }
        
        COMMTIMEOUTS commtimeouts{.ReadIntervalTimeout = 5};
        if (!SetCommTimeouts(fd, &commtimeouts)) {
            CloseHandle(fd);
            continue;
        }
        
        ++port_count;
        std::shared_ptr<pm1::chassis_t> ptr(
            new pm1::chassis_t(std::move(path)),
            [fd, &port_count, &signal](pm1::chassis_t *p) {
                delete p;
                CloseHandle(fd);
                if (!--port_count)
                    signal.notify_all();
            });
        
        // to read
        std::thread([ptr, fd] {
            read_context_t context{.chassis = ptr.get(), .handle = fd, .buffer{}, .size = 0,};
            OVERLAPPED overlapped;
            do {
                overlapped = {.hEvent = &context};
                ReadFileEx(fd, context.buffer + context.size, sizeof(context.buffer) - context.size, &overlapped, &read_callback);
            } while (SleepEx(500, true) == WAIT_IO_COMPLETION);
        }).detach();
        
        // to write
        std::thread([ptr = std::move(ptr), fd] {
            uint8_t buffer[4 * 6]{};
            #define SET_HEADER(N) *reinterpret_cast<can::header_t *>(buffer + (N)) = can::pm1
            SET_HEADER(0)::every_tcu::current_position::tx;
            SET_HEADER(6)::every_ecu::current_position::tx;
            SET_HEADER(12)::every_node::state::tx;
            SET_HEADER(18)::every_vcu::battery_percent::tx;
            #undef SET_HEADER
            for (auto i = 0; i < sizeof(buffer); i += 6)
                buffer[i + 5] = can::crc_calculate(buffer + i + 1, buffer + i + 5);
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0;; ++i) {
                using namespace std::chrono_literals;
                constexpr static auto PERIOD = 40ms;
                constexpr static uint32_t
                    N0 = 10s / PERIOD,
                    N1 = 400ms / PERIOD,
                    N2 = 2;
                auto size = i % N0 == 0
                            ? 24
                            : i % N1 == 0
                              ? 18
                              : i % N2 == 0
                                ? 12
                                : 6;
                t0 += PERIOD;
                auto overlapped = new OVERLAPPED{.hEvent = new uint8_t[size]};
                std::memcpy(overlapped->hEvent, buffer, size);
                WriteFileEx(fd, overlapped->hEvent, size, overlapped, &write_callback);
                if (SleepEx(100, true) != WAIT_IO_COMPLETION)
                    break;
                std::this_thread::sleep_until(t0);
            }
        }).detach();
    }
    
    if (port_count) {
        std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);
        signal.wait(lock);
    }
    
    return 0;
}

void WINAPI write_callback(DWORD, DWORD, LPOVERLAPPED overlapped) {
    delete[] reinterpret_cast<uint8_t *>(overlapped->hEvent);
    delete overlapped;
}

void WINAPI read_callback(DWORD error_code, DWORD actual, LPOVERLAPPED overlapped) {
    if (!error_code) {
        auto context = reinterpret_cast<read_context_t *>(overlapped->hEvent);
        auto[i, j] = context->chassis->communicate(context->buffer, context->size += actual);
        context->size = i;
        if (j > i) {
            auto size = j - i;
            overlapped = new OVERLAPPED{.hEvent = new uint8_t[size]};
            std::memcpy(overlapped->hEvent, context->buffer + i, size);
            WriteFileEx(context->handle, overlapped->hEvent, size, overlapped, &write_callback);
            SleepEx(INFINITE, true);
        }
    }
}
