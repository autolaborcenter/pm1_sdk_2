//
// Created by ydrml on 2021/3/15.
//

#ifndef PM1_SDK_2_CHASSIS_T_HH
#define PM1_SDK_2_CHASSIS_T_HH

#include "odometry_t.hpp"

extern "C" {
#include "control_model/model.h"
}

#include <chrono>

namespace autolabor::pm1 {
    class chassis_t {
        class implement_t;
        implement_t *_implement;

    public:
        chassis_t();
        chassis_t(chassis_t const &) = delete;
        chassis_t(chassis_t &&) noexcept;
        ~chassis_t();

        /// | parameter | in                     | out
        /// | --------- | ---------------------- | -
        /// | buffer    | head of received bytes | head of bytes to send
        /// | size      | size of received bytes | size of bytes to send
        void communicate(unsigned char *&buffer, unsigned char &size);

        [[nodiscard]] bool alive() const;
        [[nodiscard]] unsigned char battery_percent() const;
        [[nodiscard]] physical current() const;
        [[nodiscard]] physical target() const;
        [[nodiscard]] odometry_t<> odometry() const;

        struct active_sending_t {
            std::chrono::steady_clock::time_point time;
            uint8_t *msg;
            size_t size;
        };

        active_sending_t next_to_send();
        void set_target(physical);
        void close();
    };
}// namespace autolabor::pm1

#endif//PM1_SDK_2_CHASSIS_T_HH
