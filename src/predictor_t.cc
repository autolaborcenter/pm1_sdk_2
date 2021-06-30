#include "predictor_t.hh"

#include <atomic>
#include <utility>

namespace autolabor::pm1 {
    class predictor_t ::implement_t {
        chassis_model_t _model;

        physical _state[2];

    public:
        std::atomic<physical> _current, _target;

        implement_t()
            : _current(physical{0, 0}),
              _target(physical_zero),
              _state{{0, 0}, physical_zero} {}

        void freeze() {
            _state[0] = _current;
            _state[1] = _target;
            if (std::isnan(_state[1].rudder))
                _state[1].rudder = _state[0].rudder;
        }

        bool next(odometry_t<> &pose) {
            if (_state[0].speed == 0 && _state[1].speed == 0) return false;

            _state[0] = _model.optimize(_state[1], _state[0]);
            _state[0].rudder =
                _state[0].rudder < _state[1].rudder
                    ? std::min({_state[0].rudder + default_optimizer.period, _state[1].rudder, +pi_f / 2})
                    : std::max({_state[0].rudder - default_optimizer.period, _state[1].rudder, -pi_f / 2});

            pose += _model.to_delta(_state[0]);
            return true;
        }
    };

    predictor_t::predictor_t() : _implement(new implement_t) {}
    predictor_t::predictor_t(predictor_t &&others) noexcept : _implement(std::exchange(others._implement, nullptr)) {}
    predictor_t::~predictor_t() { delete _implement; }

    void predictor_t::set_target(physical p) { _implement->_target = p; }
    void predictor_t::set_current(physical p) { _implement->_current = p; }
    void predictor_t::freeze() { _implement->freeze(); }
    bool predictor_t::operator()(odometry_t<> &pose) { return _implement->next(pose); }
}// namespace autolabor::pm1
