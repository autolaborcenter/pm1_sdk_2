#include "../src/chassis_t.hh"

#include <Windows.h>
#include <SetupAPI.h>

#pragma comment (lib, "setupapi.lib")

#include <string>
#include <vector>
#include <unordered_map>
#include <thread>

using namespace autolabor;

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
    
    std::unordered_map<HANDLE, autolabor::pm1::chassis_t> chassis;
    for (const auto &name : ports) {
        auto p = name.find("COM");
        if (p < 0 || p >= name.size())
            continue;
        auto q = p + 2;
        while (std::isdigit(name[++q]));
        
        auto path = R"(\\.\)" + name.substr(p, q - p);
        
        auto fd = CreateFile(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (fd == INVALID_HANDLE_VALUE)
            continue;
        
        DCB dcb{.DCBlength = sizeof(DCB)};
        
        dcb.BaudRate = CBR_115200;
        dcb.ByteSize = 8;
        
        if (!SetCommState(fd, &dcb)) {
            CloseHandle(fd);
            continue;
        }
        
        chassis.try_emplace(fd, std::move(path), fd);
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(500));
    
    return 0;
}
