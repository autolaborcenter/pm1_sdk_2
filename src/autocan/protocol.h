#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <vector>

namespace autolabor::can {
    union header_t {
        uint32_t key;
        struct {
            uint8_t head;
            
            #if __BYTE_ORDER == __LITTLE_ENDIAN
            
            #define NODE_TYPE(HEADER) static_cast<uint8_t>(((HEADER).data.node_type_h << 4u) | (HEADER).data.node_type_l)
            
            uint8_t node_type_h: 2;
            uint8_t priority: 3;
            bool payload: 1;
            uint8_t network: 2;
            
            uint8_t node_index: 4;
            uint8_t node_type_l: 4;
            
            #elif __BYTE_ORDER == __BIG_ENDIAN
            
            #define NODE_TYPE(HEADER) ((HEADER).node_type)

            uint8_t network : 2;
            bool payload : 1;
            uint8_t priority : 3;
            uint8_t node_type : 6;
            uint8_t node_index : 4;
            
            #endif
            
            uint8_t msg_type;
        } data;
    };
    
    static_assert(sizeof(header_t) == 4);
    
    uint8_t crc_calculate(const uint8_t *p0, const uint8_t *end);
    
    /// byte stream -> msg pack
    std::vector<const uint8_t *> split(const uint8_t *begin, const uint8_t *end);
    
} // namespace autolabor::can

#endif // !PROTOCOL_H
