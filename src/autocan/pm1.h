//
// Created by ydrml on 2021/3/14.
//

#ifndef PM1_SDK_2_PM1_H
#define PM1_SDK_2_PM1_H

#include "protocol.h"

namespace autolabor::can::pm1 {
    #define MSG_DIALOG(MSG_TYPE, NAME) using NAME = dialog<MSG_TYPE>
    #define SPECIAL_MSG_DIALOG(CLASS, MSG_TYPE, NAME) using NAME = typename CLASS::template dialog<MSG_TYPE>
    
    template<auto _type, auto _index>
    struct node {
        constexpr static auto type = _type;
        constexpr static auto index = _index;
        
        template<auto msg_type>
        struct dialog {
            constexpr static header_t tx{.data{.head = 0xfe, .node_type_h = _type & 0b11, .payload = false, .node_index = _index, .node_type_l = _type & 0b1111, .msg_type = msg_type}};
            constexpr static header_t rx{.data{.head = 0xfe, .node_type_h = _type & 0b11, .payload = true, .node_index = _index, .node_type_l = _type & 0b1111, .msg_type = msg_type}};
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
        SPECIAL_MSG_DIALOG(vcu, 1, battery_percent);
        SPECIAL_MSG_DIALOG(vcu, 2, battery_time);
        SPECIAL_MSG_DIALOG(vcu, 3, battery_quantity);
        SPECIAL_MSG_DIALOG(vcu, 4, battery_voltage);
        SPECIAL_MSG_DIALOG(vcu, 5, battery_current);
        SPECIAL_MSG_DIALOG(vcu, 6, control_pad);
        SPECIAL_MSG_DIALOG(vcu, 7, power_switch);
    };
    
    template<auto _index>
    struct ecu : public node<0x11, _index> {
        SPECIAL_MSG_DIALOG(ecu, 5, current_speed);
        SPECIAL_MSG_DIALOG(ecu, 6, current_position);
    };
    
    template<auto _index>
    struct tcu : public node<0x12, _index> {
        SPECIAL_MSG_DIALOG(tcu, 3, current_position);
        SPECIAL_MSG_DIALOG(tcu, 5, current_speed);
    };
    
    #undef SPECIAL_MSG_DIALOG
    #undef MSG_DIALOG
}

#endif //PM1_SDK_2_PM1_H
