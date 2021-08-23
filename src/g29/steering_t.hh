#ifndef STEERING_T_HH
#define STEERING_T_HH

#include <cstdint>

namespace logitech {
    class steering_t {
        class implement_t;
        implement_t *_implement;

    public:
        static steering_t scan();

        steering_t(const char *, const char *);
        steering_t(steering_t const &) = delete;
        steering_t(steering_t &&) noexcept;
        ~steering_t();

        operator bool() const;
        void close();

        bool wait_event(uint8_t &, float &, float &, int);
    };
}// namespace logitech

#endif// STEERING_T_HH
