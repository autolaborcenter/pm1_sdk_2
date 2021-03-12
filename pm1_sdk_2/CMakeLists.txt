find_package(Threads REQUIRED)

add_library(kernel
	src/serial_port/serial_port_t.hh
	src/serial_port/serial_ports_linux.cpp
	src/serial_port/serial_port_t_linux.cc
	src/serial_port/serial_port_t_win32.cc
)

add_executable(pm1_driver_win32 "app/pm1_driver_win32.cpp")
target_link_libraries(pm1_driver_win32 kernel)
