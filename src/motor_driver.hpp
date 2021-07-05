#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>

enum MD_State {
    NO_CONTROL,
    BRAKE,
    FORWARD,
    BACK,
};

class MotorDriver {
    public:
        MotorDriver(byte in1, byte in2, byte pwm);
        void set_state(MD_State state);
        void set_duty(double);
    private:
        byte mIn1, mIn2, mPWM;
        MD_State mState;
        double mDuty;
};

#endif