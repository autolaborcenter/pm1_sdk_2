#ifndef STEERING_T_HH
#define STEERING_T_HH

class steering_t
{
    class context_t;
    context_t *_context;

public:
    static steering_t &global(const char *&);

    steering_t();
    steering_t(const char *, const char *);
    steering_t(steering_t const &) = delete;
    steering_t(steering_t &&) noexcept;
    steering_t &operator=(steering_t const &) = delete;
    steering_t &operator=(steering_t &&) noexcept;
    ~steering_t();

    bool open();
    void close();

    bool wait_event(float &, float &);
};

extern "C"
{
    bool wait_event(float &, float &);
}

#endif // STEERING_T_HH