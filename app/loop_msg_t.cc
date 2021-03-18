//
// Created by ydrml on 2021/3/18.
//

#include "loop_msg_t.hh"

#include "../src/autocan/pm1.h"

#include <chrono>

namespace autolabor::pm1 {
    loop_msg_t::loop_msg_t() : _bytes{} {
        #define SET_HEADER(N) *reinterpret_cast<can::header_t *>(_bytes + (N)) = can::pm1
        SET_HEADER(0)::every_tcu::current_position::tx;
        SET_HEADER(6)::every_ecu::current_position::tx;
        SET_HEADER(12)::every_node::state::tx;
        SET_HEADER(18)::every_vcu::battery_percent::tx;
        #undef SET_HEADER
        for (auto i = 0; i < sizeof(_bytes); i += 6)
            _bytes[i + 5] = can::crc_calculate(_bytes + i + 1, _bytes + i + 5);
    }
    
    std::pair<uint8_t *, size_t> loop_msg_t::operator[](uint64_t i) const {
        using namespace std::chrono_literals;
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
        auto buffer = new uint8_t[size];
        std::memcpy(buffer, _bytes, size);
        return {buffer, size};
    }
}
