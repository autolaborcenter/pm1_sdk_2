//
// Created by ydrml on 2021/3/14.
//

#ifndef PM1_SDK_2_PM1_H
#define PM1_SDK_2_PM1_H

#include "protocol.h"

namespace autolabor::can::pm1 {
#define MSG_DIALOG(MSG_TYPE, NAME) using NAME = dialog<MSG_TYPE>
#define MSG_DIALOG_(CLASS, MSG_TYPE, NAME) using NAME = typename CLASS::template dialog<MSG_TYPE>
#define MSG_INFO(CLASS, MSG_TYPE, NAME) constexpr static auto NAME = CLASS::template dialog<MSG_TYPE>::rx
#define MSG_CMD(CLASS, MSG_TYPE, NAME) constexpr static auto NAME = CLASS::template dialog<MSG_TYPE>::tx

    template<auto _type, auto _index>
    struct node {
        template<auto msg_type>
        struct dialog {
            constexpr static header_t
#if __BYTE_ORDER == __LITTLE_ENDIAN
                tx{.data{.head = 0xfe, .node_type_h = (_type >> 4) & 0b11, .payload = false, .node_index = _index, .node_type_l = _type & 0b1111, .msg_type = msg_type}},
                rx{.data{.head = 0xfe, .node_type_h = (_type >> 4) & 0b11, .payload = true, .node_index = _index, .node_type_l = _type & 0b1111, .msg_type = msg_type}};
#elif __BYTE_ORDER == __BIG_ENDIAN
#error
#endif
        };

        MSG_DIALOG(0x80, state);
        MSG_DIALOG(0x81, version_id);
        MSG_DIALOG(0x82, device_id);
        MSG_DIALOG(0x83, chip_id);
        MSG_DIALOG(0x84, hal_version_id);
        MSG_DIALOG(0x85, core_hardware_version_id);
        MSG_DIALOG(0x86, extra_hardware_version_id);
        MSG_DIALOG(0x87, software_version_id);
        MSG_DIALOG(0x88, up_min);
        MSG_DIALOG(0x89, up_times);
        constexpr static auto lock = dialog<0xff>::tx;
        constexpr static auto unlock = dialog<0xff>::rx;
    };

    template<auto _index>
    struct vcu : public node<0x10, _index> {
        MSG_DIALOG_(vcu, 1, battery_percent);
        MSG_DIALOG_(vcu, 2, battery_time);
        MSG_DIALOG_(vcu, 3, battery_quantity);
        MSG_DIALOG_(vcu, 4, battery_voltage);
        MSG_DIALOG_(vcu, 5, battery_current);
        MSG_DIALOG_(vcu, 6, control_pad);
        MSG_DIALOG_(vcu, 7, power_switch);
        MSG_INFO(vcu, 8, target_speed);
    };

    template<auto _index>
    struct ecu : public node<0x11, _index> {
        MSG_INFO(ecu, 1, target_speed);
        MSG_DIALOG_(ecu, 5, current_speed);
        MSG_DIALOG_(ecu, 6, current_position);
        MSG_CMD(ecu, 7, encoder_reset);
        MSG_INFO(ecu, 10, command_timeout);
    };

    template<auto _index>
    struct tcu : public node<0x12, _index> {
        MSG_INFO(tcu, 1, target_position);
        MSG_INFO(tcu, 2, increment_position);
        MSG_DIALOG_(tcu, 3, current_position);
        MSG_INFO(tcu, 4, target_speed);
        MSG_DIALOG_(tcu, 5, current_speed);
        MSG_CMD(tcu, 6, set_zero);
        MSG_INFO(tcu, 7, command_timeout);
    };

    using every_node = node<0x3f, 0x0f>;
    using every_vcu = vcu<0x0f>;
    using every_ecu = ecu<0x0f>;
    using every_tcu = tcu<0x0f>;

    using any_node = node<0, 0>;
    using any_vcu = vcu<0>;
    using any_ecu = ecu<0>;
    using any_tcu = tcu<0>;

#undef SPECIAL_MSG_DIALOG
#undef MSG_DIALOG
}// namespace autolabor::can::pm1

#endif//PM1_SDK_2_PM1_H
