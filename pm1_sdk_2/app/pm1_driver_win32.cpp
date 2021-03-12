#include "../src/serial_port/serial_port_t.hh"

#include <Windows.h>
#include <SetupAPI.h>

#pragma comment (lib, "setupapi.lib")

#include <string>
#include <vector>
#include <iostream>

int main() {
	std::vector<std::string> ports;
	{
		auto set = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

		if (set == INVALID_HANDLE_VALUE)
			return 1;

		SP_DEVINFO_DATA data{ .cbSize = sizeof(SP_DEVINFO_DATA) };
		char buffer[64];
		for (auto i = 0; SetupDiEnumDeviceInfo(set, i, &data); ++i) {
			SetupDiGetDeviceRegistryProperty(set, &data, SPDRP_FRIENDLYNAME, NULL, reinterpret_cast<PBYTE>(buffer), sizeof(buffer), NULL);
			ports.emplace_back(buffer);
		}

		SetupDiDestroyDeviceInfoList(set);
	}

	for (const auto& name : ports) {
		auto p = name.find("COM");
		if (p < 0 || p >= name.size())
			continue;
		auto q = p + 2;
		while (std::isdigit(name[++q]));
		serial_port_t port(name.substr(p, q - p));
		std::cout << name << ": " << std::boolalpha << (port.open() == 0) << std::endl;
	}
	
	return 0;
}
