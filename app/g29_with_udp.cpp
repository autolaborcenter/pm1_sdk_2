#include "g29/steering_t.hh"
extern "C" {
#include "control_model/model.h"
}

#include <arpa/inet.h>
#include <linux/socket.h>
#include <netinet/in.h>

#include <iostream>

int main() {
    auto udp = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in remote{.sin_family = AF_INET, .sin_port = 33333};
    inet_pton(AF_INET, "6.0.0.3", &remote.sin_addr);

    float speed, rudder;
    while (wait_event(speed, rudder, -1)) {
        std::cout << speed << " | " << rudder << std::endl;
        physical temp{speed, rudder};
        sendto(udp, &temp, sizeof(temp), MSG_WAITALL, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
    }
    return 0;
}
