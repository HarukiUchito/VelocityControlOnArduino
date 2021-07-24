#include "speedController.hpp"

SpeedController::SpeedController(
    MotorDriver* md_ptr,
    Encoder* enc_ptr,
    double gainP, double gainI, double gainD
) : mMD(md_ptr), mEncoder(enc_ptr),
    mP(gainP), mI(gainI), mD(gainD),
    mIntegralSum(0.0), mRPS(0.0),
    targetVelocity_(0.0), currentVelocity_(0.0)
{

}

void SpeedController::setMotorDuty(double duty)
{
    if (duty < 0.0)
        mMD->set_state(MD_State::BACK);
    else if (duty > 0.0)
        mMD->set_state(MD_State::FORWARD);
    else
        mMD->set_state(MD_State::BRAKE);
    
    mMD->set_duty(abs(duty));
}

void SpeedController::controlVelocity()
{
    double lastRPS = mRPS;
    mRPS = abs(mEncoder->get_rps());
    double dt = mEncoder->get_dt() / 1000.0;

    // ramp acceleration
    double max_acc = 0.1; // r/s^2
    double e_vel = targetVelocity_ - currentVelocity_;
    double ctrl = (e_vel < 0.0) ? 
        min(max_acc * dt, e_vel) : max(-max_acc * dt, e_vel);
    currentVelocity_ += ctrl;

    // speed PID control
    double e_rps = abs(currentVelocity_) - mRPS;
    double nduty = 0.0;
    nduty += mP * e_rps;

    mIntegralSum += e_rps * dt;
    nduty += mI * mIntegralSum;

    if (dt == 0.0)
        dt = 1;
    nduty += mD * (mRPS - lastRPS) / dt;

    nduty = min(nduty, 100.0);
    nduty = max(nduty, 0.0);

    // switch direction
    if (currentVelocity_ < 0.0)
        mMD->set_state(MD_State::BACK);
    else if (currentVelocity_ > 0.0)
        mMD->set_state(MD_State::FORWARD);
    else
        mMD->set_state(MD_State::BRAKE);
    
    mMD->set_duty(nduty);
}

void SpeedController::setVelocity(double velocity)
{
    targetVelocity_ = velocity;
}
