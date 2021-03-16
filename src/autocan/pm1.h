//
// Created by ydrml on 2021/3/14.
//

#ifndef PM1_SDK_2_PM1_H
#define PM1_SDK_2_PM1_H

#include "protocol.h"

namespace autolabor::can::pm1 {
    #define MSG_DIALOG(MSG_TYPE, NAME) using NAME = dialog<MSG_TYPE>
    #define _MSG_DIALOG(CLASS, MSG_TYPE, NAME) using NAME = typename CLASS::template dialog<MSG_TYPE>
    #define _MSG_TX(CLASS, MSG_TYPE, NAME) constexpr static auto NAME = CLASS::template dialog<MSG_TYPE>::tx
    
    template<auto _type, auto _index>
    struct node {
        template<auto msg_type>
        struct dialog {
            constexpr static header_t
            #if __BYTE_ORDER == __LITTLE_ENDIAN
                tx{.data{.head = 0xfe, .node_type_h = _type & 0b11, .payload = false, .node_index = _index, .node_type_l = _type & 0b1111, .msg_type = msg_type}},
                rx{.data{.head = 0xfe, .node_type_h = _type & 0b11, .payload = true, .node_index = _index, .node_type_l = _type & 0b1111, .msg_type = msg_type}};
            #elif __BYTE_ORDER == __BIG_ENDIAN
            tx{.data{.payload = false, .node_type = _type, .node_index = _index, .msg_type = msg_type}},
            rx{.data{.payload = true, .node_type = _type, .node_index = _index, .msg_type = msg_type}};
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
        MSG_DIALOG(0x88, uptime_id);
    };
    
    template<auto _index>
    struct vcu : public node<0x10, _index> {
        _MSG_DIALOG(vcu, 1, battery_percent);
        _MSG_DIALOG(vcu, 2, battery_time);
        _MSG_DIALOG(vcu, 3, battery_quantity);
        _MSG_DIALOG(vcu, 4, battery_voltage);
        _MSG_DIALOG(vcu, 5, battery_current);
        _MSG_DIALOG(vcu, 6, control_pad);
        _MSG_DIALOG(vcu, 7, power_switch);
        _MSG_TX(vcu, 8, target_speed);
    };
    
    template<auto _index>
    struct ecu : public node<0x11, _index> {
        _MSG_TX(ecu, 1, target_speed);
        _MSG_DIALOG(ecu, 5, current_speed);
        _MSG_DIALOG(ecu, 6, current_position);
        _MSG_TX(ecu, 7, encoder_reset);
        _MSG_TX(ecu, 10, command_timeout);
    };
    
    template<auto _index>
    struct tcu : public node<0x12, _index> {
        _MSG_TX(tcu, 1, target_position);
        _MSG_TX(tcu, 2, increment_position);
        _MSG_DIALOG(tcu, 3, current_position);
        _MSG_TX(tcu, 4, target_speed);
        _MSG_DIALOG(tcu, 5, current_speed);
        _MSG_TX(tcu, 6, set_zero);
        _MSG_TX(tcu, 7, command_timeout);
    };
    
    #undef SPECIAL_MSG_DIALOG
    #undef MSG_DIALOG
}

#endif //PM1_SDK_2_PM1_H
