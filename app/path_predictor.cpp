#include "src/predictor_t.hh"

#include <iostream>
#include <sstream>

int main() {
    autolabor::pm1::predictor_t predictor;
    std::string line;

    while (std::getline(std::cin, line)) {
        std::stringstream builder(line);
        char head;
        physical p;
        float speed, rudder;
        if (!(builder >> head >> p.speed >> p.rudder)) continue;
        switch (head) {
            case 'C':
                predictor.set_current(p);
                break;
            case 'T':
                predictor.set_target(p);
                predictor.freeze();
                {
                    autolabor::odometry_t<> pose{}, delta{};
                    std::cout << "P 0,0,0";
                    for (auto i = 0;
                         i < 500 &&
                         std::abs(pose.theta) < pi_f * 2 &&
                         std::hypotf(pose.x, pose.y) < 3 &&
                         predictor(delta);
                         ++i)
                        if (std::abs(delta.s) > .05f) {
                            pose += delta.as_delta();
                            delta = {};
                            std::cout << ' ' << pose.x << ',' << pose.y << ',' << pose.theta;
                        }
                    std::cout << std::endl;
                }
                break;
        }
    }

    return 0;
}
