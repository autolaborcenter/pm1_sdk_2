//
// Created by ydrml on 2021/3/15.
//

#ifndef PM1_SDK_2_CHASSIS_T_HH
#define PM1_SDK_2_CHASSIS_T_HH

namespace autolabor::pm1 {
    class chassis_t {
        class implement_t;

        implement_t *_implement;

    public:
        chassis_t();

        ~chassis_t();

        /// | parameter | in                     | out
        /// | --------- | ---------------------- | -
        /// | buffer    | head of received bytes | head of bytes to send
        /// | size      | size of received bytes | size of bytes to send
        void communicate(unsigned char *&buffer, unsigned char &size);

        [[nodiscard]] bool alive() const;

        [[nodiscard]] unsigned char battery_percent() const;

        void close();

        void state(float &speed, float &rudder) const;

        void set_physical(float &speed, float &rudder);
    };
}// namespace autolabor::pm1

#endif//PM1_SDK_2_CHASSIS_T_HH
