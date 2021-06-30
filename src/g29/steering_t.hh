#ifndef STEERING_T_HH
#define STEERING_T_HH

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

    bool wait_event(float &, float &, int);
};

#endif// STEERING_T_HH
