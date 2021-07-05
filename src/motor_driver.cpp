#include "motor_driver.hpp"

MotorDriver::MotorDriver(byte in1, byte in2, byte pwm) :
    mState(NO_CONTROL),
    mDuty(0.0)
{
    mIn1 = in1;
    mIn2 = in2;
    mPWM = pwm;
}

void MotorDriver::set_state(MD_State state)
{
    if (mState == state)
        return;
    mState = state;
    switch (mState)
    {
    case NO_CONTROL:
        digitalWrite(mIn1, 0);
        digitalWrite(mIn2, 0);
        break;
    case BRAKE:
        digitalWrite(mIn1, 1);
        digitalWrite(mIn2, 1);
        break;
    case FORWARD:
        digitalWrite(mIn1, 1);
        digitalWrite(mIn2, 0);
        break;
    case BACK:
        digitalWrite(mIn1, 0);
        digitalWrite(mIn2, 1);
        break;
    default:
        break;
    }
}

void MotorDriver::set_duty(double duty)
{
    mDuty = duty;
    if (mState == NO_CONTROL || mState == BRAKE)
        mDuty = 0.0;
    analogWrite(mPWM, mDuty * 255.0 / 100.0);
}
