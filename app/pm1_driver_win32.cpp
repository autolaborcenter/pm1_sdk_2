#include "../src/chassis_t.hh"
#include "loop_msg_t.hh"

#include <Windows.h>
#include <SetupAPI.h>

#pragma comment (lib, "setupapi.lib")

#include <unordered_map>
#include <mutex>
#include <thread>

using namespace autolabor::pm1;

static void WINAPI write_callback(DWORD, DWORD, LPOVERLAPPED);

static void WINAPI read_callback(DWORD, DWORD, LPOVERLAPPED);

struct read_context_t {
    chassis_t *chassis;
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
    
    std::unordered_map<std::string, chassis_t> chassis;
    std::mutex mutex;
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
        
        auto chassis_ptr = &chassis.try_emplace(path).first->second;
        std::shared_ptr<HANDLE> fd_ptr(
            new HANDLE(fd),
            [path, &chassis, &mutex, &signal](HANDLE *p) {
                CloseHandle(*p);
                delete p;
                std::unique_lock<std::mutex> lock(mutex);
                chassis.erase(path);
                if (chassis.empty()) {
                    lock.unlock();
                    signal.notify_all();
                }
            });
        
        std::thread([chassis_ptr, fd_ptr] {
            read_context_t context{.chassis = chassis_ptr, .handle = *fd_ptr, .buffer{}, .size = 0,};
            OVERLAPPED overlapped;
            do {
                overlapped = {.hEvent = &context};
                ReadFileEx(*fd_ptr, context.buffer + context.size, sizeof(context.buffer) - context.size, &overlapped, &read_callback);
            } while (SleepEx(500, true) == WAIT_IO_COMPLETION);
            chassis_ptr->close();
        }).detach();
        
        std::thread([chassis_ptr, fd_ptr] {
            auto msg = loop_msg_t();
            auto t0 = std::chrono::steady_clock::now();
            for (uint64_t i = 0; chassis_ptr->alive(); ++i) {
                auto[buffer, size] = msg[i];
                t0 += loop_msg_t::PERIOD;
                auto overlapped = new OVERLAPPED{.hEvent = buffer};
                WriteFileEx(*fd_ptr, overlapped->hEvent, size, overlapped, &write_callback);
                if (SleepEx(loop_msg_t::PERIOD.count(), true) != WAIT_IO_COMPLETION)
                    break;
                std::this_thread::sleep_until(t0);
            }
            chassis_ptr->close();
        }).detach();
    }
    
    std::unique_lock<std::mutex> lock(mutex);
    while (!chassis.empty()) signal.wait(lock);
    
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
