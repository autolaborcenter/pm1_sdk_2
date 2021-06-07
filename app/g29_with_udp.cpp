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

    physical last, target;
    while (wait_event(target.speed, target.rudder, 50)) {
        std::cout << target.speed << " | " << target.rudder << std::endl;
        if (!target.speed && target.speed == last.speed && target.rudder == last.rudder)
            continue;
        last = target;
        sendto(udp, &target, sizeof(target), MSG_WAITALL, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
    }
    return 0;
}
