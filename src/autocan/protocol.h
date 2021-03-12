#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>

namespace autolabor::can
{
    struct header_t
    {
        uint8_t head;

#if __BYTE_ORDER == __LITTLE_ENDIAN

#define NODE_TYPE(HEADER) static_cast<uint8_t>((HEADER.node_type_h << 4u) | HEADER.node_type_l)

        uint8_t node_type_h : 2;
        uint8_t priority : 3;
        bool payload : 1;
        uint8_t network : 2;

        uint8_t node_index : 4;
        uint8_t node_type_l : 4;

#elif __BYTE_ORDER == __BIG_ENDIAN

#define NODE_TYPE(HEADER) (HEADER.node_type)

        uint8_t network : 2;
        bool payload : 1;
        uint8_t priority : 3;
        uint8_t node_type : 6;
        uint8_t node_index : 4;

#endif

        uint8_t msg_type;
        uint8_t frame_id;
    };

    static_assert(sizeof(header_t) == 5);

} // namespace autolabor::can

#endif // !PROTOCOL_H
