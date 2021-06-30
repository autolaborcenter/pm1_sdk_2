#ifndef PREDICTOR_HH
#define PREDICTOR_HH

#include "chassis_model_t.hh"

namespace autolabor::pm1 {
    class predictor_t {
        class implement_t;
        implement_t *_implement;

    public:
        predictor_t();
        predictor_t(predictor_t const &) = delete;
        predictor_t(predictor_t &&) noexcept;
        ~predictor_t();

        void set_target(physical);
        void set_current(physical);
        void freeze();
        bool operator()(odometry_t<> &);
    };
}// namespace autolabor::pm1

#endif// PREDICTOR_HH
