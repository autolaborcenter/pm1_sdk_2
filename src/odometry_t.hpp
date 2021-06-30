//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_2_ODOMETRY_T_HPP
#define PM1_SDK_2_ODOMETRY_T_HPP

#include <cmath>

namespace autolabor {
    enum class odometry_type : bool {
        state,
        delta
    };

    template<odometry_type type = odometry_type::state, class t = float>
    struct odometry_t;

    template<class t = float>
    using odometry_delta_t = odometry_t<odometry_type::delta, t>;

    /** 里程计增量 */
    template<class t>
    struct odometry_t<odometry_type::delta, t> {
        t s, a, x, y, theta;

        static odometry_t from_velocity(t distance, t theta) {
            auto a = std::abs(theta);

            if (a < std::numeric_limits<t>::epsilon())
                return {std::abs(distance), a, distance, 0, theta};

            auto r = distance / theta;
            return {std::abs(distance), a, r * std::sin(theta), r * (1 - std::cos(theta)), theta};
        }
    };

    /** 里程计状态 */
    template<class t>
    struct odometry_t<odometry_type::state, t> {
        t s, a, x, y, theta;

        odometry_t &operator+=(const odometry_t<odometry_type::delta> &delta) {
            s += delta.s;
            a += delta.a;

            const auto sin = std::sin(theta),
                       cos = std::cos(theta);
            theta += delta.theta;

            x += delta.x * cos - delta.y * sin;
            y += delta.x * sin + delta.y * cos;
            return *this;
        }

        odometry_t &operator-=(const odometry_t<odometry_type::delta> &delta) {
            s -= delta.s;
            a -= delta.a;

            theta -= delta.theta;
            const auto sin = std::sin(theta),
                       cos = std::cos(theta);

            x -= delta.x * cos - delta.y * sin;
            y -= delta.x * sin + delta.y * cos;
            return *this;
        }

        odometry_t operator+(const odometry_t<odometry_type::delta> &delta) const {
            auto temp = *this;
            return temp += delta;
        }

        odometry_t operator-(const odometry_t<odometry_type::delta> &delta) const {
            auto temp = *this;
            return temp -= delta;
        }

        odometry_t<odometry_type::delta> operator-(const odometry_t<odometry_type::state> &mark) const {
            const auto sin = std::sin(-mark.theta),
                       cos = std::cos(-mark.theta),
                       dx = x - mark.x,
                       dy = y - mark.y;

            return {s - mark.s,
                    a - mark.a,
                    dx * cos - dy * sin,
                    dx * sin + dy * cos,
                    theta - mark.theta};
        }
    };
}// namespace autolabor

#endif//PM1_SDK_2_ODOMETRY_T_HPP
